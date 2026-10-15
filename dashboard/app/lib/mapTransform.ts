/**
 * Conversions between the saved map image and the robots' map frame.
 *
 * The map image follows ROS map_server conventions: `origin` is the world
 * position (metres) of the image's bottom-left corner, `resolution` is metres
 * per pixel, and image rows count down from the top while world y counts up.
 * Pixel coordinates here are in image pixels (not screen pixels) and may be
 * fractional.
 */
export type MapMeta = {
  resolution: number;
  origin: [number, number];
  widthPx: number;
  heightPx: number;
};

export function pixelToWorld(px: number, py: number, meta: MapMeta): { x: number; y: number } {
  return {
    x: meta.origin[0] + px * meta.resolution,
    y: meta.origin[1] + (meta.heightPx - py) * meta.resolution,
  };
}

export function worldToPixel(x: number, y: number, meta: MapMeta): { px: number; py: number } {
  return {
    px: (x - meta.origin[0]) / meta.resolution,
    py: meta.heightPx - (y - meta.origin[1]) / meta.resolution,
  };
}

export function isInsideMap(px: number, py: number, meta: MapMeta): boolean {
  return px >= 0 && py >= 0 && px < meta.widthPx && py < meta.heightPx;
}
