use heapless::Vec;

pub struct RingBuf<const CAP: usize> {
    data: [u8; CAP],
    start: usize,
    len: usize,
}

impl<const CAP: usize> Default for RingBuf<CAP> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const CAP: usize> RingBuf<CAP> {
    pub const fn new() -> Self {
        assert!(CAP > 0, "RingBuf capacity must be non-zero");
        Self {
            data: [0u8; CAP],
            start: 0,
            len: 0,
        }
    }

    /// Append `bytes` to the buffer.
    ///
    /// When the buffer is full, oldest bytes are overwritten to make room.
    /// If `bytes` is longer than `CAP`, only the last `CAP` bytes are kept.
    pub fn push_bytes(&mut self, bytes: &[u8]) {
        if bytes.len() >= CAP {
            self.start = 0;
            self.len = CAP;
            self.data.copy_from_slice(&bytes[bytes.len() - CAP..]);
            return;
        }
        let write_pos = (self.start + self.len) % CAP;
        let first_chunk_len = CAP - write_pos;
        if bytes.len() <= first_chunk_len {
            self.data[write_pos..write_pos + bytes.len()].copy_from_slice(bytes);
        } else {
            self.data[write_pos..].copy_from_slice(&bytes[..first_chunk_len]);
            self.data[..bytes.len() - first_chunk_len].copy_from_slice(&bytes[first_chunk_len..]);
        }
        self.len += bytes.len();
        if self.len > CAP {
            let overflow = self.len - CAP;
            self.start = (self.start + overflow) % CAP;
            self.len = CAP;
        }
    }

    /// Drain up to N bytes from the front of the buffer.
    pub fn drain<const N: usize>(&mut self) -> Vec<u8, N> {
        let n = self.len.min(N);
        let mut out: Vec<u8, N> = Vec::new();
        // SAFETY: n <= N (Vec capacity) so set_len is in-bounds. The n bytes
        // exposed as uninitialized are fully overwritten by the copy_from_slice
        // calls below before out is returned.
        unsafe { out.set_len(n) };
        let first_len = (CAP - self.start).min(n);
        out[..first_len].copy_from_slice(&self.data[self.start..self.start + first_len]);
        out[first_len..].copy_from_slice(&self.data[..n - first_len]);
        self.start = (self.start + n) % CAP;
        self.len -= n;
        out
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
}

impl<const CAP: usize> core::fmt::Write for RingBuf<CAP> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.push_bytes(s.as_bytes());
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn basic_push_and_drain() {
        let mut rb: RingBuf<16> = RingBuf::new();
        rb.push_bytes(b"hello");
        let out: Vec<u8, 16> = rb.drain();
        assert_eq!(&out[..], b"hello");
        assert!(rb.is_empty());
    }

    #[test]
    fn drain_less_than_available() {
        let mut rb: RingBuf<16> = RingBuf::new();
        rb.push_bytes(b"abcdef");
        let out: Vec<u8, 3> = rb.drain();
        assert_eq!(&out[..], b"abc");
        let rest: Vec<u8, 16> = rb.drain();
        assert_eq!(&rest[..], b"def");
    }

    #[test]
    fn drain_empty_returns_empty() {
        let mut rb: RingBuf<16> = RingBuf::new();
        let out: Vec<u8, 16> = rb.drain();
        assert!(out.is_empty());
    }

    #[test]
    fn overflow_drops_oldest_bytes() {
        let mut rb: RingBuf<4> = RingBuf::new();
        rb.push_bytes(b"abcd");
        rb.push_bytes(b"ef");
        let out: Vec<u8, 4> = rb.drain();
        assert_eq!(&out[..], b"cdef");
    }

    #[test]
    fn overflow_input_larger_than_capacity() {
        let mut rb: RingBuf<4> = RingBuf::new();
        rb.push_bytes(b"abcdefgh");
        let out: Vec<u8, 4> = rb.drain();
        assert_eq!(&out[..], b"efgh");
    }

    #[test]
    fn wrap_around() {
        let mut rb: RingBuf<4> = RingBuf::new();
        rb.push_bytes(b"abc");
        let _: Vec<u8, 2> = rb.drain(); // drain 2, start moves to 2
        // after drain(2): start=2, len=1 (c remains at index 2)
        // push "de": write_pos=3, first_len=1 ('d' at 3), then 'e' wraps to index 0
        rb.push_bytes(b"de");
        let out: Vec<u8, 4> = rb.drain();
        assert_eq!(&out[..], b"cde");
    }

    #[test]
    fn fill_exactly_then_drain() {
        let mut rb: RingBuf<4> = RingBuf::new();
        rb.push_bytes(b"abcd");
        assert!(!rb.is_empty());
        let out: Vec<u8, 4> = rb.drain();
        assert_eq!(&out[..], b"abcd");
        assert!(rb.is_empty());
    }

    #[test]
    fn multiple_cycles() {
        let mut rb: RingBuf<8> = RingBuf::new();
        for i in 0..4u8 {
            rb.push_bytes(&[i]);
            let out: Vec<u8, 1> = rb.drain();
            assert_eq!(out[0], i);
        }
        assert!(rb.is_empty());
    }

    #[test]
    fn overflow_into_wrapped_buffer() {
        // start != 0 when overflow happens, so the start pointer must advance correctly
        let mut rb: RingBuf<4> = RingBuf::new();
        rb.push_bytes(b"ab");
        let _: Vec<u8, 1> = rb.drain(); // start=1, len=1 ('b' at index 1)
        rb.push_bytes(b"cd"); // write_pos=2, fills [2,3]: start=1, len=3 (b,c,d)
        rb.push_bytes(b"ef"); // overflows by 1: start advances to 2, len=4 (c,d,e,f)
        let out: Vec<u8, 4> = rb.drain();
        assert_eq!(&out[..], b"cdef");
    }

    #[test]
    fn fmt_write() {
        use core::fmt::Write as _;
        let mut rb: RingBuf<32> = RingBuf::new();
        let _ = write!(rb, "val={}", 42u32);
        let out: Vec<u8, 32> = rb.drain();
        assert_eq!(&out[..], b"val=42");
    }
}
