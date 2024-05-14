// Copyright © SixtyFPS GmbH <info@slint.dev>
// SPDX-License-Identifier: MIT

#![cfg_attr(not(feature = "std"), no_std)]
extern crate alloc;

#[cfg(feature = "esp32-s3-box")]
mod esp32_s3_box;
#[cfg(feature = "esp32-s3-box")]
pub mod gt911;
#[cfg(feature = "esp32-s3-box")]
pub use esp32_s3_box::*;

#[cfg(not(any(
    feature = "esp32-s3-box"
)))]
pub use i_slint_core_macros::identity as entry;

#[cfg(not(any(
    feature = "esp32-s3-box"
)))]
pub fn init() {}
