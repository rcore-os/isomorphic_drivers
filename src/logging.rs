//! Logging macros for isomorphic_driver

#[cfg(feature = "log")]
#[macro_use]
mod log {
    macro_rules! drv_debug {
        ($($arg:expr),*) => (debug!($($arg),*));
    }
    macro_rules! drv_info {
        ($($arg:expr),*) => (info!($($arg),*));
    }
    macro_rules! drv_warn {
        ($($arg:expr),*) => (warn!($($arg),*));
    }
}

#[cfg(not(feature = "log"))]
#[macro_use]
mod log {
    macro_rules! drv_debug {
        ($($arg:expr),*) => { $( let _ = $arg; )* };
    }
    macro_rules! drv_info {
        ($($arg:expr),*) => { $( let _ = $arg; )*};
    }
    macro_rules! drv_warn {
        ($($arg:expr),*) => { $( let _ = $arg; )*};
    }
}
