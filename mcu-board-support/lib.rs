// Copyright © SixtyFPS GmbH <info@slint.dev>
// SPDX-License-Identifier: MIT

#![cfg_attr(not(feature = "std"), no_std)]
extern crate alloc;

#[cfg(feature = "esp32-s3-box-3")]
mod esp32_s3_box_3;
#[cfg(feature = "esp32-s3-box-3")]
pub mod gt911;
#[cfg(feature = "esp32-s3-box-3")]
pub use esp32_s3_box_3::*;

#[cfg(not(any(
    feature = "esp32-s3-box-3"
)))]
pub use i_slint_core_macros::identity as entry;

#[cfg(not(any(
    feature = "esp32-s3-box-3"
)))]
pub fn init() {}
