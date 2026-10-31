// ブラウザ画面の確認(M3)。juiz で app/app.conf を起動しておき、Playwright で開いて操作する
// 使い方: node scripts/ui_test.js [http://localhost:8000/] [スクリーンショットの保存先]
const { chromium } = require("playwright");

const url = process.argv[2] || "http://localhost:8000/";
const shot = process.argv[3] || "ui_test.png";

function check(cond, msg) {
  if (!cond) { console.error("FAILED: " + msg); process.exit(1); }
  console.log("ok -", msg);
}

(async () => {
  const browser = await chromium.launch();
  const page = await browser.newPage({ viewport: { width: 1000, height: 800 } });
  const errors = [];
  page.on("pageerror", (e) => errors.push(e.message));
  await page.goto(url);
  await page.waitForSelector("#connection.ok", { timeout: 10000 });
  check(true, "connected to juiz");
  await page.waitForFunction(() => document.getElementById("s_pose").textContent !== "-");

  const pose = async () => {
    const t = await page.textContent("#s_pose");
    const m = t.match(/x (-?[\d.]+), y (-?[\d.]+), θ (-?[\d.]+)/);
    return { x: Number(m[1]), y: Number(m[2]), th: Number(m[3]) };
  };

  // マップの中央(0, 0)をクリックしてロボットを置く(ドラッグで +x 向き)
  const box = await page.locator("#map").boundingBox();
  const cx = box.x + box.width / 2, cy = box.y + box.height / 2;
  await page.mouse.move(cx, cy);
  await page.mouse.down();
  await page.mouse.move(cx + 40, cy, { steps: 4 });
  await page.mouse.up();
  await page.waitForTimeout(400);
  let p = await pose();
  check(Math.abs(p.x) < 0.1 && Math.abs(p.y) < 0.1 && Math.abs(p.th) < 0.05, `placed at the center facing +x (${JSON.stringify(p)})`);

  // W を押し続けると前に進む
  await page.keyboard.down("w");
  await page.waitForTimeout(1500);
  await page.keyboard.up("w");
  await page.waitForTimeout(300);
  const p2 = await pose();
  check(p2.x > p.x + 0.2, `W key moves the robot forward (${JSON.stringify(p2)})`);

  // A で左に旋回する
  await page.keyboard.down("a");
  await page.waitForTimeout(1000);
  await page.keyboard.up("a");
  await page.waitForTimeout(300);
  const p3 = await pose();
  check(p3.th > p2.th + 0.2, `A key turns left (${JSON.stringify(p3)})`);

  // パッドの上側をドラッグすると前に進む
  const pad = await page.locator("#pad").boundingBox();
  await page.mouse.move(pad.x + pad.width / 2, pad.y + pad.height / 2);
  await page.mouse.down();
  await page.mouse.move(pad.x + pad.width / 2, pad.y + 10, { steps: 3 });
  await page.waitForTimeout(1000);
  await page.mouse.up();
  await page.waitForTimeout(300);
  const p4 = await pose();
  check(Math.hypot(p4.x - p3.x, p4.y - p3.y) > 0.1, `pad moves the robot (${JSON.stringify(p4)})`);

  // LiDAR の光線を出してスクリーンショット
  await page.check("#show_rays");
  await page.waitForTimeout(300);
  await page.screenshot({ path: shot });
  check(errors.length === 0, `no page errors ${JSON.stringify(errors)}`);
  await browser.close();
})().catch((e) => { console.error(e); process.exit(1); });
