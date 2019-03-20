#![feature(alloc)]
#![feature(const_fn)]
#![no_std]
#![allow(unused_variables, dead_code)]

extern crate alloc;
#[cfg(feature = "log")]
#[macro_use]
extern crate log;

#[macro_use]
mod logging;

pub mod net;
pub mod provider;