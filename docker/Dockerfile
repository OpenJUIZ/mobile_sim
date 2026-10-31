# syntax=docker/dockerfile:1
# mobile_sim のイメージ。リポジトリのルートを context にしてビルドする:
#   docker build -f docker/Dockerfile -t mobile_sim .
#
# Rust のプラグインは juiz と同じ juiz_sdk・同じ rustc でないと読み込めないので、
# juiz とプラグインを同じ builder でビルドする(juiz の版は Cargo.toml の juiz_sdk と揃える)。
ARG JUIZ_VERSION=0.0.29

FROM rust:1-bookworm AS build
ARG JUIZ_VERSION
# 埋め込みの Python は使わないので外す
RUN --mount=type=cache,target=/usr/local/cargo/registry \
    cargo install juiz_app --version "${JUIZ_VERSION}" --locked --no-default-features
WORKDIR /src
COPY . .
RUN --mount=type=cache,target=/usr/local/cargo/registry \
    --mount=type=cache,target=/src/target \
    grep -qx "juiz_sdk = \"=${JUIZ_VERSION}\"" Cargo.toml \
    && cargo build -p mobile_sim --release --locked \
    && target=$(rustc -vV | sed -n 's/^host: //p') \
    && mkdir -p "app/plugins/$target" \
    && cp target/release/libmobile_sim.so "app/plugins/$target/"

FROM debian:bookworm-slim
COPY --from=build /usr/local/cargo/bin/juiz /usr/local/bin/juiz
COPY --from=build /src/app /opt/mobile_sim
COPY LICENSE-MIT LICENSE-APACHE /opt/mobile_sim/
# app.conf の相対パス(地図、ブラウザ画面)はここから解決される
WORKDIR /opt/mobile_sim
# 別の地図は -v ./maps:/maps -e MOBILE_SIM_MAP=/maps/<mapconf>.yaml で渡す
VOLUME ["/maps"]
EXPOSE 8000
ENTRYPOINT ["juiz"]
CMD ["-f", "app.conf", "-d"]
