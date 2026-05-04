use arrayvec::ArrayVec;
use deranged::RangedI16;
use lf_hal_types::line_sensor::SensorReadings;

const POSITION_RANGE: i16 = 1024;
const MIN_STRENGTH: u16 = 1000;

/// Sensors must exceed this fraction of the peak anomaly to join a cluster.
const CLUSTER_THRESHOLD_PCT: i32 = 30;

#[derive(Clone, Copy, Debug)]
pub struct LineDetection {
    /// Center of the detection. Negative left, positive right.
    pub position: RangedI16<{ -POSITION_RANGE }, POSITION_RANGE>,
    /// Total cluster anomaly. TODO: Figure out and specify range
    pub strength: u16,
}

fn push_detection(detections: &mut ArrayVec<LineDetection, 2>, new: LineDetection) {
    if new.strength < MIN_STRENGTH {
        return;
    }
    if let Err(err) = detections.try_push(new) {
        let new = err.element();
        if let Some(weakest) = detections.iter_mut().min_by_key(|d| d.strength) {
            if new.strength > weakest.strength {
                *weakest = new;
            }
        }
    }
}

pub fn detect_line(readings: &SensorReadings) -> ArrayVec<LineDetection, 2> {
    let readings = &readings.values;
    let n = readings.len();

    let max = readings.iter().copied().map(i32::from).max().unwrap();
    let min = readings.iter().copied().map(i32::from).min().unwrap();

    let cluster_threshold = (max - min) * CLUSTER_THRESHOLD_PCT / 100;

    let mut result = ArrayVec::new();

    // Maps weighted centroid index to output position range.
    let cluster_position = |centroid_sum: i32, sum: i32| -> RangedI16<-1024, 1024> {
        debug_assert!(centroid_sum.unsigned_abs() < 1 << 19);
        let position = centroid_sum * (POSITION_RANGE as i32 * 2) / (sum * (n - 1) as i32)
            - POSITION_RANGE as i32;
        (position as i16).try_into().unwrap()
    };

    let mut cluster_count = 0usize;
    let mut cluster_sum = 0i32;
    let mut cluster_centroid_sum = 0i32;

    for (i, &reading) in readings.iter().enumerate() {
        debug_assert!(
            reading.unsigned_abs() < 4096,
            "Difference between two 12bit ADC readings (ambient light subtraction)"
        );

        let anomaly = max - reading as i32;

        if anomaly > cluster_threshold {
            cluster_count += 1;
            cluster_sum += anomaly;
            cluster_centroid_sum += anomaly * i as i32;
        } else if cluster_count > 0 {
            push_detection(
                &mut result,
                LineDetection {
                    position: cluster_position(cluster_centroid_sum, cluster_sum),
                    strength: cluster_sum as u16,
                },
            );
            cluster_count = 0;
            cluster_sum = 0;
            cluster_centroid_sum = 0;
        }
    }

    if cluster_count > 0 {
        push_detection(
            &mut result,
            LineDetection {
                position: cluster_position(cluster_centroid_sum, cluster_sum),
                strength: cluster_sum as u16,
            },
        );
    }

    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use lf_hal_types::line_sensor::SENSOR_COUNT;

    const BG: i16 = 3000;
    const DARK: i16 = 1000;

    fn with_dark(positions: &[usize]) -> SensorReadings {
        let mut values = [BG; SENSOR_COUNT];
        for &p in positions {
            values[p] = DARK;
        }
        SensorReadings { values }
    }

    #[test]
    fn uniform_is_no_line() {
        assert!(
            detect_line(&SensorReadings {
                values: [BG; SENSOR_COUNT]
            })
            .is_empty()
        );
    }

    #[test]
    fn low_contrast_is_no_line() {
        let mut values = [BG; SENSOR_COUNT];
        values[4] = BG - 100;
        assert!(detect_line(&SensorReadings { values }).is_empty());
    }

    #[test]
    fn dark_line_center() {
        let result = detect_line(&with_dark(&[4, 5]));
        assert_eq!(result.len(), 1);
        assert!(
            result[0].position.get().abs() < 154,
            "position {}",
            result[0].position.get()
        );
        assert!(result[0].strength > 0);
    }

    #[test]
    fn dark_line_left() {
        let result = detect_line(&with_dark(&[1]));
        assert_eq!(result.len(), 1);
        assert!(
            result[0].position.get() < -512,
            "position {}",
            result[0].position.get()
        );
    }

    #[test]
    fn dark_line_right() {
        let result = detect_line(&with_dark(&[8]));
        assert_eq!(result.len(), 1);
        assert!(
            result[0].position.get() > 512,
            "position {}",
            result[0].position.get()
        );
    }

    #[test]
    fn dark_line_leftmost_edge() {
        let result = detect_line(&with_dark(&[0]));
        assert_eq!(result.len(), 1);
        assert!(
            result[0].position.get() < -819,
            "position {}",
            result[0].position.get()
        );
    }

    #[test]
    fn dark_line_rightmost_edge() {
        let result = detect_line(&with_dark(&[9]));
        assert_eq!(result.len(), 1);
        assert!(
            result[0].position.get() > 819,
            "position {}",
            result[0].position.get()
        );
    }

    #[test]
    fn wide_dark_line_is_single_line() {
        let result = detect_line(&with_dark(&[3, 4, 5, 6]));
        assert_eq!(result.len(), 1);
        assert!(
            result[0].position.get().abs() < 154,
            "position {}",
            result[0].position.get()
        );
    }

    #[test]
    fn split_line() {
        let result = detect_line(&with_dark(&[1, 2, 7, 8]));
        assert_eq!(result.len(), 2);
        assert!(
            result[0].position.get() < 0,
            "left {}",
            result[0].position.get()
        );
        assert!(
            result[1].position.get() > 0,
            "right {}",
            result[1].position.get()
        );
    }

    #[test]
    fn position_monotone_left_to_right() {
        let mut positions = [0i16; SENSOR_COUNT];
        for i in 0..SENSOR_COUNT {
            let result = detect_line(&with_dark(&[i]));
            assert_eq!(result.len(), 1, "expected 1 detection at index {}", i);
            positions[i] = result[0].position.get();
        }
        for i in 0..SENSOR_COUNT - 1 {
            assert!(
                positions[i] < positions[i + 1],
                "not monotone at {}: {} >= {}",
                i,
                positions[i],
                positions[i + 1]
            );
        }
    }
}
