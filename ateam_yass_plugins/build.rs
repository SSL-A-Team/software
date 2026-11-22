use std::{path::Path, process::Command};


fn resolve_submodule_git_dir(submodule_path: &Path) -> Option<std::path::PathBuf> {
    let output = Command::new("git")
        .args(["rev-parse", "--git-dir"])
        .current_dir(submodule_path)
        .output()
        .ok()?;

    if !output.status.success() {
        return None;
    }

    let git_dir_str = String::from_utf8(output.stdout).ok()?.trim().to_string();
    let git_dir = Path::new(&git_dir_str);
    Some(if git_dir.is_absolute() {
        git_dir.to_path_buf()
    } else {
        submodule_path.join(git_dir)
    })
}

fn check_git_status(path: &Path) -> (String, bool) {
    let sha = Command::new("git")
        .args(["rev-parse", "HEAD"])
        .current_dir(&path)
        .output()
        .ok()
        .filter(|o| o.status.success())
        .and_then(|o| String::from_utf8(o.stdout).ok())
        .map(|s| s.trim().to_string())
        .unwrap();

    let is_dirty = Command::new("git")
        .args(["status", "--porcelain"])
        .current_dir(&path)
        .output()
        .ok()
        .filter(|o| o.status.success())
        .map(|o| !o.stdout.is_empty())
        .unwrap();

    println!("cargo:rerun-if-changed={}", path.display());
    if let Some(git_dir) = resolve_submodule_git_dir(&path) {
        println!("cargo:rerun-if-changed={}", git_dir.join("HEAD").display());
        println!("cargo:rerun-if-changed={}", git_dir.join("index").display());
    }

    (sha, is_dirty)
}

fn main() {
    let crate_dir = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    let submodule_path = Path::new(&crate_dir).join("../radio/ateam_software_communication/software-communication");
    let (comms_sha, comms_dirty) = check_git_status(&submodule_path);
    println!("cargo:rustc-env=COMMS_GIT_SHA={comms_sha}");
    println!("cargo:rustc-env=COMMS_GIT_DIRTY={comms_dirty}");
}
