// Copy the parts of three.js the page needs into dist/vendor so the app runs
// from any static file server with no CDN and no bundler.
import { cpSync, mkdirSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const src = join(root, "node_modules", "three");
const out = join(root, "dist", "vendor", "three");

mkdirSync(out, { recursive: true });
cpSync(join(src, "build", "three.module.js"), join(out, "build", "three.module.js"));
for (const dir of ["controls", "utils"]) {
  cpSync(join(src, "examples", "jsm", dir), join(out, "examples", "jsm", dir), { recursive: true });
}
console.log("vendored three.js into dist/vendor");
