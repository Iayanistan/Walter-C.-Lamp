const { chromium } = require('playwright');
const fs = require('fs');
(async () => {
  const [svgPath, pngPath, scale] = process.argv.slice(2);
  const svg = fs.readFileSync(svgPath, 'utf8');
  const b = await chromium.launch();
  const p = await b.newPage({ viewport: { width: 1600, height: 1150 }, deviceScaleFactor: Number(scale || 2) });
  await p.setContent(`<html><body style="margin:0">${svg}</body></html>`);
  await p.screenshot({ path: pngPath });
  await b.close();
})();
