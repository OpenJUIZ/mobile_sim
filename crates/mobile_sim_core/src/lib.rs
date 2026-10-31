//! mobile_sim のシミュレーション本体。
//!
//! 車両モデル、マップと mapconf、衝突判定、LiDAR のレイキャストを持つ(M1、issue #2)。
//! juiz には依存せず、`cargo test` だけで検証できるようにする。

/// このクレートの版
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn version_is_set() {
        assert!(!VERSION.is_empty());
    }
}
