use core::cell::RefCell;
use core::fmt::Write as _;
use core::task::{Poll, Waker};
use critical_section::Mutex;
use embassy_time::Instant;
use heapless::Vec;

use crate::utilities::ring_buf::RingBuf;

struct LoggerInternal {
    buf: RingBuf<1024>,
    waker: Option<Waker>,
}

struct BleLogger {
    inner: Mutex<RefCell<LoggerInternal>>,
}

impl BleLogger {
    const fn new() -> Self {
        Self {
            inner: Mutex::new(RefCell::new(LoggerInternal {
                buf: RingBuf::new(),
                waker: None,
            })),
        }
    }

    async fn read_log_bytes<const N: usize>(&self) -> Vec<u8, N> {
        core::future::poll_fn(|cx| {
            critical_section::with(|cs| {
                let mut inner = self.inner.borrow(cs).borrow_mut();
                let data: Vec<u8, N> = inner.buf.drain();
                if !data.is_empty() {
                    return Poll::Ready(data);
                }
                inner.waker = Some(cx.waker().clone());
                Poll::Pending
            })
        })
        .await
    }
}

fn format_record(record: &log::Record) -> heapless::String<256> {
    static LEVEL_CHARS: [char; 6] = [' ', 'E', 'W', 'I', 'D', 'T'];
    let micros = Instant::now().as_micros();
    let mut s = heapless::String::new();
    let _ = write!(
        s,
        "{:>3}.{:06} {} {}",
        micros / 1_000_000,
        micros % 1_000_000,
        LEVEL_CHARS[record.level() as usize],
        record.args()
    );
    s
}

impl log::Log for BleLogger {
    fn enabled(&self, metadata: &log::Metadata) -> bool {
        if metadata.target().starts_with("esp_rtos") {
            metadata.level() <= log::Level::Warn
        } else {
            true
        }
    }

    fn log(&self, record: &log::Record) {
        if !self.enabled(record.metadata()) {
            return;
        }

        let formatted = format_record(record);

        esp_println::println!("{formatted}");
        let waker = critical_section::with(|cs| {
            let mut inner = self.inner.borrow(cs).borrow_mut();
            inner.buf.push_bytes(formatted.as_bytes());
            inner.buf.push_bytes(b"\n");
            // Waking goes through the esp-rtos scheduler borrow path. Calling it
            // from within the scheduler (which logs under the "esp_rtos" target)
            // causes a re-entrant borrow panic, so we skip the wake in that case.
            // Those messages still land in the buffer and on UART.
            if record.target().starts_with("esp_rtos") {
                None
            } else {
                inner.waker.take()
            }
        });
        if let Some(w) = waker {
            w.wake();
        }
    }

    fn flush(&self) {}
}

static LOGGER: BleLogger = BleLogger::new();

pub fn init() {
    log::set_logger(&LOGGER).ok();
    log::set_max_level(log::LevelFilter::Debug);
}

pub async fn read_log_bytes<const N: usize>() -> Vec<u8, N> {
    LOGGER.read_log_bytes().await
}
