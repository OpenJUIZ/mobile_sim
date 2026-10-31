"""juiz で app/app.conf を起動し、HTTP からプロセスを呼んで動作を確かめる(M2 の結合テスト)

使い方: scripts/dev_plugin.sh && python3 scripts/integration_test.py [--juiz <juiz のパス>] [--port 18000]
       python3 scripts/integration_test.py --juizapp dist/mobile_sim-0.1.0.juizapp   # juiz run で起動する(ポートは 8000)
"""
import os
import argparse
import json
import subprocess
import sys
import tempfile
import time
import urllib.parse
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--juiz", default="juiz")
    ap.add_argument("--port", type=int, default=18000)
    ap.add_argument("--juizapp", help="test this package with `juiz run` instead of app/app.conf")
    args = ap.parse_args()
    log = tempfile.TemporaryFile()
    tmp = None

    if args.juizapp:
        # 基礎パッケージのポートは juizapp.yaml の base.http_port(8000)で決まる。
        # カレントディレクトリと展開先のキャッシュは空の一時ディレクトリにする(app/ の中身に頼らないことを確かめる)
        base = "http://localhost:8000"
        work = tempfile.mkdtemp()
        env = dict(os.environ, JUIZ_CACHE_DIR=os.path.join(work, "cache"))
        cmd = [args.juiz, "run", str(Path(args.juizapp).resolve())]
        cwd = work
    else:
        # ポートだけ変えた conf を app/ に置く(プラグインと地図の相対パスを保つため)
        base = f"http://localhost:{args.port}"
        conf = (ROOT / "app" / "app.conf").read_text()
        conf = conf.replace('"port": 8000', f'"port": {args.port}')
        tmp = ROOT / "app" / ".integration_test.conf"
        tmp.write_text(conf)
        env = None
        cmd = [args.juiz, "-f", tmp.name, "-d"]
        cwd = ROOT / "app"

    proc = subprocess.Popen(cmd, cwd=cwd, env=env, stdout=log, stderr=subprocess.STDOUT)
    try:
        wait_ready(base, proc, log)
        run_checks(base)
        print("OK")
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
        if tmp:
            tmp.unlink(missing_ok=True)


def wait_ready(base, proc, log):
    for _ in range(150):
        if proc.poll() is not None:
            log.seek(0)
            sys.exit("juiz exited:\n" + log.read().decode(errors="replace"))
        try:
            with urllib.request.urlopen(f"{base}/api/container/list", timeout=1) as r:
                if json.load(r):
                    return
        except OSError:
            pass
        time.sleep(0.2)
    sys.exit("juiz did not become ready")


def call(base, name, kind, args=None, raw=False):
    """コンテナ sim0 の ContainerProcess `name`(型 mobile_sim_<kind>)を呼ぶ"""
    ident = f"{base}/container_process/{name}::mobile_sim_{kind}:sim0"
    url = f"{base}/api/container_process/call?identifier=" + urllib.parse.quote(ident, safe="")
    req = urllib.request.Request(
        url,
        data=json.dumps(args or {}).encode(),
        method="PATCH",
        headers={"Content-Type": "application/json"},
    )
    with urllib.request.urlopen(req, timeout=5) as r:
        body = r.read()
        return (r.headers.get("Content-Type"), body) if raw else json.loads(body)


def velocity(vx, omega=0.0):
    return {"velocity": {"linear": {"x": vx, "y": 0.0}, "angular": omega}}


def check(cond, msg):
    if not cond:
        sys.exit("FAILED: " + msg)
    print("ok -", msg)


def run_checks(base):
    p0 = call(base, "get_pose0", "get_pose")["pose"]
    check(abs(p0["position"]["x"] + 2.0) < 1e-6, f"initial pose x = -2 ({p0})")

    for _ in range(10):
        call(base, "set_velocity0", "set_velocity", velocity(0.5))
        time.sleep(0.1)
    p1 = call(base, "get_pose0", "get_pose")["pose"]
    check(p1["position"]["x"] > p0["position"]["x"] + 0.1, f"set_velocity moves the robot ({p1})")

    time.sleep(1.5)
    odo = call(base, "get_odometry0", "get_odometry")
    check(odo["twist"]["vx"] == 0.0, f"robot stops after command_timeout ({odo['twist']})")
    check(odo["distance"] > 0.1, "odometry distance accumulates")

    scan = call(base, "get_scan0", "get_scan")
    check(len(scan["ranges"]) == 271, "scan has 271 beams")
    check(all(r <= scan["range_max"] + 1.0 for r in scan["ranges"]), "scan ranges are finite")

    info = call(base, "get_map_info0", "get_map_info")
    check((info["width"], info["height"], info["resolution"]) == (200, 200, 0.05), "map info")

    ctype, png = call(base, "get_map0", "get_map", raw=True)
    check(png[:4] == b"\x89PNG", f"get_map returns PNG ({ctype})")

    call(base, "reset_pose0", "reset_pose", {"pose": {"position": {"x": 1.0, "y": 1.0}, "theta": 0.5}})
    p2 = call(base, "get_pose0", "get_pose")["pose"]
    check(abs(p2["position"]["x"] - 1.0) < 1e-6 and abs(p2["theta"] - 0.5) < 1e-6, "reset_pose")

    # 仕切り壁(x = 1.5)へ向けて走らせると、手前で止まる
    call(base, "reset_pose0", "reset_pose", {"pose": {"position": {"x": 0.5, "y": 3.0}, "theta": 0.0}})
    for _ in range(30):
        call(base, "set_velocity0", "set_velocity", velocity(1.0))
        time.sleep(0.1)
    st = call(base, "get_state0", "get_state")["robots"][0]
    check(st["pose"]["x"] < 1.5 - 0.2 + 1e-6, f"robot stops before the wall ({st['pose']})")
    check(st["bumper"], "bumper is set at the wall")


if __name__ == "__main__":
    main()
