//! Loader and checker for the MARV vehicle card and environment file.
//! All the parsing and checking lives in `schema.rs` (std, `toml`, `sha2` only).

mod schema;

pub use schema::*;
