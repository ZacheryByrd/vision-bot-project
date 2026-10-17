import { describe, expect, it } from "vitest";
import { isInsideMap, pixelToWorld, worldToPixel, type MapMeta } from "./mapTransform";

const meta: MapMeta = { resolution: 0.05, origin: [-5, -5], widthPx: 200, heightPx: 200 };

describe("pixelToWorld", () => {
  it("maps the bottom-left pixel corner to the origin", () => {
    expect(pixelToWorld(0, 200, meta)).toEqual({ x: -5, y: -5 });
  });

  it("maps the top-right corner to the far corner", () => {
    expect(pixelToWorld(200, 0, meta)).toEqual({ x: 5, y: 5 });
  });

  it("maps the image centre to the world origin", () => {
    expect(pixelToWorld(100, 100, meta)).toEqual({ x: 0, y: 0 });
  });
});

describe("worldToPixel", () => {
  it.each([
    [0, 200],
    [200, 0],
    [100, 100],
  ])("inverts pixelToWorld for (%i, %i)", (px, py) => {
    const { x, y } = pixelToWorld(px, py, meta);
    const back = worldToPixel(x, y, meta);
    expect(back.px).toBeCloseTo(px, 9);
    expect(back.py).toBeCloseTo(py, 9);
  });
});

describe("isInsideMap", () => {
  it("rejects points left of or below the image", () => {
    expect(isInsideMap(-1, 10, meta)).toBe(false);
    expect(isInsideMap(10, 201, meta)).toBe(false);
  });

  it("accepts points on the image", () => {
    expect(isInsideMap(0, 0, meta)).toBe(true);
    expect(isInsideMap(199.5, 199.5, meta)).toBe(true);
  });
});
