//! The RTL-SDR through librtlsdr (LGPL, dynamically linked): the ten calls
//! the radio needs, wrapped so the rest of the program never sees a raw
//! pointer. librtlsdr detaches the kernel DVB driver itself.

use anyhow::{anyhow, bail, Result};
use std::ffi::{c_char, c_int, c_void, CStr};

#[repr(C)]
pub struct RtlsdrDev {
    _private: [u8; 0],
}

#[link(name = "rtlsdr")]
extern "C" {
    fn rtlsdr_get_device_count() -> u32;
    fn rtlsdr_get_device_name(index: u32) -> *const c_char;
    fn rtlsdr_get_device_usb_strings(
        index: u32,
        manufact: *mut c_char,
        product: *mut c_char,
        serial: *mut c_char,
    ) -> c_int;
    fn rtlsdr_get_index_by_serial(serial: *const c_char) -> c_int;
    fn rtlsdr_open(dev: *mut *mut RtlsdrDev, index: u32) -> c_int;
    fn rtlsdr_close(dev: *mut RtlsdrDev) -> c_int;
    fn rtlsdr_set_sample_rate(dev: *mut RtlsdrDev, rate: u32) -> c_int;
    fn rtlsdr_set_center_freq(dev: *mut RtlsdrDev, freq: u32) -> c_int;
    fn rtlsdr_set_tuner_gain_mode(dev: *mut RtlsdrDev, manual: c_int) -> c_int;
    fn rtlsdr_get_tuner_gains(dev: *mut RtlsdrDev, gains: *mut c_int) -> c_int;
    fn rtlsdr_set_tuner_gain(dev: *mut RtlsdrDev, gain: c_int) -> c_int;
    fn rtlsdr_set_agc_mode(dev: *mut RtlsdrDev, on: c_int) -> c_int;
    fn rtlsdr_set_bias_tee(dev: *mut RtlsdrDev, on: c_int) -> c_int;
    fn rtlsdr_reset_buffer(dev: *mut RtlsdrDev) -> c_int;
    fn rtlsdr_read_async(
        dev: *mut RtlsdrDev,
        cb: extern "C" fn(*mut u8, u32, *mut c_void),
        ctx: *mut c_void,
        buf_num: u32,
        buf_len: u32,
    ) -> c_int;
    fn rtlsdr_cancel_async(dev: *mut RtlsdrDev) -> c_int;
}

pub struct Device {
    dev: *mut RtlsdrDev,
    /// Supported tuner gains, tenths of a dB, ascending.
    pub gains: Vec<i32>,
    pub name: String,
}

unsafe impl Send for Device {}

/// One entry of the device list.
pub struct Listing {
    pub index: u32,
    pub name: String,
    pub serial: String,
}

pub fn list() -> Vec<Listing> {
    let n = unsafe { rtlsdr_get_device_count() };
    (0..n)
        .map(|i| {
            let name = unsafe { CStr::from_ptr(rtlsdr_get_device_name(i)) }
                .to_string_lossy()
                .into_owned();
            let mut m = [0 as c_char; 256];
            let mut p = [0 as c_char; 256];
            let mut s = [0 as c_char; 256];
            let serial = if unsafe {
                rtlsdr_get_device_usb_strings(i, m.as_mut_ptr(), p.as_mut_ptr(), s.as_mut_ptr())
            } == 0
            {
                unsafe { CStr::from_ptr(s.as_ptr()) }
                    .to_string_lossy()
                    .into_owned()
            } else {
                String::new()
            };
            Listing {
                index: i,
                name,
                serial,
            }
        })
        .collect()
}

impl Device {
    /// Open by index, or by serial when `serial` is given.
    pub fn open(index: u32, serial: Option<&str>) -> Result<Device> {
        let index = match serial {
            Some(s) => {
                let cs = std::ffi::CString::new(s)?;
                let i = unsafe { rtlsdr_get_index_by_serial(cs.as_ptr()) };
                if i < 0 {
                    bail!("no RTL-SDR with serial {s}");
                }
                i as u32
            }
            None => index,
        };
        if unsafe { rtlsdr_get_device_count() } == 0 {
            bail!("no RTL-SDR found on USB");
        }
        let mut dev: *mut RtlsdrDev = std::ptr::null_mut();
        let r = unsafe { rtlsdr_open(&mut dev, index) };
        if r != 0 || dev.is_null() {
            bail!("rtlsdr_open({index}) failed: {r} (in use, or no permission on /dev/bus/usb)");
        }
        let name = unsafe { CStr::from_ptr(rtlsdr_get_device_name(index)) }
            .to_string_lossy()
            .into_owned();
        let n = unsafe { rtlsdr_get_tuner_gains(dev, std::ptr::null_mut()) };
        let mut gains = vec![0 as c_int; n.max(0) as usize];
        if n > 0 {
            unsafe { rtlsdr_get_tuner_gains(dev, gains.as_mut_ptr()) };
        }
        Ok(Device { dev, gains, name })
    }

    pub fn set_sample_rate(&mut self, hz: u32) -> Result<()> {
        check(
            unsafe { rtlsdr_set_sample_rate(self.dev, hz) },
            "set_sample_rate",
        )
    }

    pub fn set_center_freq(&mut self, hz: u32) -> Result<()> {
        check(
            unsafe { rtlsdr_set_center_freq(self.dev, hz) },
            "set_center_freq",
        )
    }

    /// Manual gain in tenths of a dB, snapped to the nearest supported step.
    pub fn set_gain(&self, tenths_db: i32) -> Result<i32> {
        let g = *self
            .gains
            .iter()
            .min_by_key(|&&g| (g - tenths_db).abs())
            .ok_or_else(|| anyhow!("tuner reports no gain steps"))?;
        check(
            unsafe { rtlsdr_set_tuner_gain_mode(self.dev, 1) },
            "set_tuner_gain_mode",
        )?;
        check(
            unsafe { rtlsdr_set_tuner_gain(self.dev, g) },
            "set_tuner_gain",
        )?;
        Ok(g)
    }

    /// Hardware automatic gain (tuner AGC and RTL2832 AGC).
    pub fn set_agc(&self) -> Result<()> {
        check(
            unsafe { rtlsdr_set_tuner_gain_mode(self.dev, 0) },
            "set_tuner_gain_mode",
        )?;
        check(unsafe { rtlsdr_set_agc_mode(self.dev, 1) }, "set_agc_mode")
    }

    /// Bias tee: 4.5 V on the antenna port for a powered LNA. Off unless
    /// asked; feeding it into a passive antenna can damage it.
    pub fn set_bias_tee(&mut self, on: bool) -> Result<()> {
        check(
            unsafe { rtlsdr_set_bias_tee(self.dev, on as c_int) },
            "set_bias_tee",
        )
    }

    pub fn reset_buffer(&mut self) -> Result<()> {
        check(unsafe { rtlsdr_reset_buffer(self.dev) }, "reset_buffer")
    }

    /// Stream samples until `cancel` is called from inside `f` or the
    /// device fails. librtlsdr keeps `buf_num` USB transfers of `buf_len`
    /// bytes queued, so the dongle's FIFO never waits on the host; a
    /// synchronous read leaves the bus idle between transfers and the
    /// RTL2832U silently drops samples in that gap (1.5 % of the stream on
    /// a Pi 3B, enough to defeat MLAT clock sync). `f` runs on this thread
    /// for every completed transfer and must return promptly.
    pub fn run(&self, buf_num: u32, buf_len: u32, mut f: impl FnMut(&[u8], &Device)) -> Result<()> {
        struct Ctx<'a> {
            dev: &'a Device,
            f: &'a mut dyn FnMut(&[u8], &Device),
        }
        extern "C" fn trampoline(buf: *mut u8, len: u32, ctx: *mut c_void) {
            let ctx = unsafe { &mut *(ctx as *mut Ctx) };
            let bytes = unsafe { std::slice::from_raw_parts(buf, len as usize) };
            (ctx.f)(bytes, ctx.dev);
        }
        let mut ctx = Ctx { dev: self, f: &mut f };
        let r = unsafe {
            rtlsdr_read_async(
                self.dev,
                trampoline,
                &mut ctx as *mut Ctx as *mut c_void,
                buf_num,
                buf_len,
            )
        };
        if r != 0 {
            bail!("rtlsdr_read_async failed: {r} (USB error)");
        }
        Ok(())
    }

    /// Stop `run`; it returns after the queued transfers are cancelled.
    pub fn cancel(&self) {
        unsafe { rtlsdr_cancel_async(self.dev) };
    }
}

impl Drop for Device {
    fn drop(&mut self) {
        unsafe { rtlsdr_close(self.dev) };
    }
}

fn check(r: c_int, what: &str) -> Result<()> {
    if r == 0 {
        Ok(())
    } else {
        Err(anyhow!("{what} failed: {r}"))
    }
}
