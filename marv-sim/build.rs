//! Records which firmware checkout the sim was built against, so the runner can write it into the log.

use std::path::Path;
use std::process::Command;

const FIRMWARE: &str = "../../MARV-V2-firmware";

fn git(args: &[&str]) -> Option<String> {
    let out = Command::new("git").arg("-C").arg(FIRMWARE).args(args).output().ok()?;
    if !out.status.success() {
        return None;
    }
    Some(String::from_utf8_lossy(&out.stdout).trim().to_string())
}

fn main() {
    // "unknown" and dirty, if git cannot say: the log then shows that the firmware is not pinned down.
    let commit = git(&["rev-parse", "HEAD"]).unwrap_or_else(|| "unknown".to_string());
    let dirty = git(&["status", "--porcelain"]).is_none_or(|s| !s.is_empty());
    println!("cargo:rustc-env=FW_COMMIT={commit}");
    println!("cargo:rustc-env=FW_DIRTY={}", if dirty { 1 } else { 0 });

    // A commit or a staged change shows in these two files. An edit to a file that was already modified does not,
    // so the two source folders are watched as well.
    for path in [".git/HEAD", ".git/index", "crates/fw-flight/src", "crates/fw-core/src"] {
        println!("cargo:rerun-if-changed={}", Path::new(FIRMWARE).join(path).display());
    }
}
