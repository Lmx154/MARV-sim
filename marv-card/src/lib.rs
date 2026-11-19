//! The MARV vehicle card. Always: the `const` data that build.rs makes from marv.toml (no_std).
//! With feature `std`: the loader and the checks, all in `schema.rs` (std, `toml`, `sha2` only).
#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(feature = "std")]
mod schema;

#[cfg(feature = "std")]
pub use schema::*;

include!(concat!(env!("OUT_DIR"), "/card_consts.rs"));
