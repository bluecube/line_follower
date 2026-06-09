#![no_std]
#![no_main]

extern crate alloc;

use alloc::vec;

use embassy_executor::Spawner;
use embassy_time::Instant;
use esp_backtrace as _;

esp_bootloader_esp_idf::esp_app_desc!();

// The requested 800x600 8-bit frame is 469 KB and does not fit in the ~160 KB
// heap. We benchmark a smaller resident frame instead so the whole 8-bit
// grayscale image stays in memory and can be inspected.
const WIDTH: usize = 320;
const HEIGHT: usize = 240;
const MAX_ITER: u16 = 256;

// Complex-plane window. Width spans the classic [-2.5, 1.0] range; the heigh t is
// derived from it so the aspect ratio matches the pixel grid.
const X_MIN: f32 = -2.5;
const X_MAX: f32 = 1.0;
const Y_CENTER: f32 = 0.0;

/// Number of escape iterations for the point `c = re + i*im`, capped at
/// [`MAX_ITER`]. Points that never escape (inside the set) return `MAX_ITER`.
fn escape_iterations(re: f32, im: f32) -> u16 {
    let mut zr = 0.0f32;
    let mut zi = 0.0f32;
    let mut iter = 0u16;
    while iter < MAX_ITER {
        let zr2 = zr * zr;
        let zi2 = zi * zi;
        if zr2 + zi2 > 4.0 {
            break;
        }
        zi = 2.0 * zr * zi + im;
        zr = zr2 - zi2 + re;
        iter += 1;
    }
    iter
}

/// Maps an escape count to an 8-bit grayscale value: points inside the set are
/// black (0), points that escape quickly are bright.
fn grayscale(iter: u16) -> u8 {
    if iter >= MAX_ITER {
        0
    } else {
        (iter * 255 / MAX_ITER) as u8
    }
}

fn render(frame: &mut [u8]) {
    let x_step = (X_MAX - X_MIN) / WIDTH as f32;
    // Keep pixels square by reusing the horizontal step for the vertical axis.
    let y_span = x_step * HEIGHT as f32;
    let y_min = Y_CENTER - y_span / 2.0;

    for py in 0..HEIGHT {
        let im = y_min + py as f32 * x_step;
        let row = &mut frame[py * WIDTH..(py + 1) * WIDTH];
        for (px, pixel) in row.iter_mut().enumerate() {
            let re = X_MIN + px as f32 * x_step;
            *pixel = grayscale(escape_iterations(re, im));
        }
    }
}

#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    let _hal = line_follower::init!(_spawner);

    let mut frame = vec![0u8; WIDTH * HEIGHT];

    log::info!(
        "Mandelbrot {}x{} 8-bit grayscale, max {} iterations",
        WIDTH,
        HEIGHT,
        MAX_ITER
    );

    let start = Instant::now();
    render(&mut frame);
    let elapsed = start.elapsed();

    // Simple checksum so the optimizer cannot discard the rendered frame.
    let checksum: u32 = frame.iter().map(|&p| p as u32).sum();

    let pixels = (WIDTH * HEIGHT) as u64;
    let micros = elapsed.as_micros();
    let pixels_per_sec = if micros > 0 {
        pixels * 1_000_000 / micros
    } else {
        0
    };

    log::info!("Render time: {} ms ({} us)", elapsed.as_millis(), micros);
    log::info!("Throughput:  {} pixels/s", pixels_per_sec);
    log::info!("Checksum:    {}", checksum);

    dump_frame(&frame);

    loop {}
}

/// Streams the rendered frame over UART as an ASCII (P2) PGM image, one pixel
/// row per line. The block is a complete, loadable PGM file: capture the serial
/// output on the host and save the `P2`...last-row span to a `.pgm`.
fn dump_frame(frame: &[u8]) {
    esp_println::println!("P2");
    esp_println::println!("{WIDTH} {HEIGHT}");
    esp_println::println!("255");

    for row in frame.chunks(WIDTH) {
        for &pixel in row {
            esp_println::print!("{pixel} ");
        }
        esp_println::println!();
    }
}
