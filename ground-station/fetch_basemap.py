"""발사장 위성영상을 미리 받아 캐시로 저장한다.

인터넷이 되는 곳에서 실행하면 basemaps/<이름>.png / .json이 만들어지고,
지상국은 현장에서 이 파일만 읽는다 (네트워크 사용 없음).

    python fetch_basemap.py site1               # 고흥만 항공센터
    python fetch_basemap.py site2               # POSTECH
    python fetch_basemap.py site1 --span 3000   # 범위 넓게
    python fetch_basemap.py mysite --lat 34.61 --lon 127.21 --label "예비 발사장"

받은 타일은 tilecache/에 그대로 남는다. 중심을 조금 옮겨 다시 받을 때
겹치는 타일은 네트워크를 타지 않고 캐시에서 바로 읽으므로, 위치를 미세
조정해도 새로 필요한 부분만 내려받는다.

출처: VWorld(국토교통부) 위성영상. 인증키 없이 열려 있는 타일 엔드포인트를 쓴다.
"""
import argparse
import json
import math
import os
import sys
import time
import urllib.error
import urllib.request

# 윈도우 기본 콘솔(cp949/cp1252)에서 한글 출력이 죽지 않도록
try:
    sys.stdout.reconfigure(encoding="utf-8")
except (AttributeError, OSError):
    pass

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
TILE_PX = 256
TILE_URL = "https://xdworld.vworld.kr/2d/Satellite/service/{z}/{x}/{y}.jpeg"

BASEMAP_DIR = os.path.join(BASE_DIR, "basemaps")
TILE_CACHE_DIR = os.path.join(BASE_DIR, "tilecache")

DEFAULT_SPAN = 2000.0        # 반경 1km
DEFAULT_ZOOM = 19            # VWorld 위성 최대 해상도 (0.246 m/px)

# 미리 등록해둔 지점 - 이름만 주면 좌표를 찾아 쓴다
SITES = {
    "site1": {
        "lat": 34.610556,        # 34°36'38.0"N
        "lon": 127.211472,       # 127°12'41.3"E
        "label": "고흥만 항공센터",
    },
    "site2": {
        "lat": 36.013800,
        "lon": 129.322200,
        "label": "POSTECH 풍동동",
    },
}

# GL 텍스처 크기 한계. RGBA로 메모리에 올라가므로 한 변이 커지면 급격히 무거워진다
# (4096^2 = 67MB, 6144^2 = 151MB, 8192^2 = 268MB)
WARN_MOSAIC_PX = 4096        # 이 이상이면 경고만
MAX_MOSAIC_PX = 8704         # 이 이상이면 중단 (GL_MAX_TEXTURE_SIZE는 보통 16384)

# VWorld 위성영상은 z19가 최대다 (z20 이상은 404). z19 = 약 0.25 m/px
MAX_ZOOM = 19


def tile_float(lat, lon, zoom):
    """위경도 -> 소수점 타일 좌표 (Web Mercator, 표준 슬리피 타일)."""
    n = 2.0 ** zoom
    x = (lon + 180.0) / 360.0 * n
    y = (1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n
    return x, y


def meters_per_pixel(lat, zoom):
    """해당 위도에서 픽셀 하나가 덮는 실제 지상 거리(m)."""
    return 156543.033928 / (2 ** zoom) * math.cos(math.radians(lat))


def fetch_tile(x, y, zoom, retries=3):
    """타일 하나를 가져온다. (bytes, 캐시적중여부)

    한 번 받은 타일은 tilecache/에 남겨 다음 실행에서 재사용한다.
    중심을 옮겨 다시 받을 때 겹치는 영역은 네트워크를 타지 않는다.
    """
    cache_path = os.path.join(TILE_CACHE_DIR, str(zoom), str(x), f"{y}.jpeg")
    if os.path.isfile(cache_path):
        try:
            with open(cache_path, "rb") as handle:
                return handle.read(), True
        except OSError:
            pass                      # 캐시가 깨졌으면 다시 받는다

    url = TILE_URL.format(z=zoom, x=x, y=y)
    request = urllib.request.Request(url, headers={
        "User-Agent": "PSI-GroundStation/1.0",
        "Referer": "https://vworld.kr/",
    })
    for attempt in range(retries):
        try:
            with urllib.request.urlopen(request, timeout=20) as response:
                data = response.read()
            os.makedirs(os.path.dirname(cache_path), exist_ok=True)
            with open(cache_path, "wb") as handle:
                handle.write(data)
            return data, False
        except (urllib.error.URLError, TimeoutError) as error:
            if attempt == retries - 1:
                print(f"    타일 {x},{y} 실패: {error}")
                return None, False
            time.sleep(0.5 * (attempt + 1))
    return None, False


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("name", nargs="?", default="site1",
                        help=f"지점 이름. 등록됨: {', '.join(SITES)}")
    parser.add_argument("--lat", type=float)
    parser.add_argument("--lon", type=float)
    parser.add_argument("--label", help="GUI에 표시할 이름")
    parser.add_argument("--span", type=float, default=DEFAULT_SPAN,
                        help=f"한 변 길이(m). 기본 {DEFAULT_SPAN:.0f} = 반경 1km")
    parser.add_argument("--zoom", type=int, default=DEFAULT_ZOOM,
                        help=f"타일 줌. {MAX_ZOOM}=0.25m/px(최대), 18=0.5m/px, 17=1m/px")
    parser.add_argument("--out", default=BASEMAP_DIR)
    args = parser.parse_args()

    if args.zoom > MAX_ZOOM:
        print(f"중단: VWorld 위성영상은 z{MAX_ZOOM}가 최대입니다 (z{args.zoom}는 404).")
        return 1

    preset = SITES.get(args.name, {})
    lat = args.lat if args.lat is not None else preset.get("lat")
    lon = args.lon if args.lon is not None else preset.get("lon")
    label = args.label or preset.get("label") or args.name
    if lat is None or lon is None:
        print(f"중단: '{args.name}'은 등록되지 않은 이름입니다. "
              "--lat/--lon을 주거나 등록된 이름을 쓰세요.")
        print(f"  등록됨: {', '.join(SITES)}")
        return 1
    args.lat, args.lon = lat, lon
    os.makedirs(args.out, exist_ok=True)

    # PyQt는 JPEG 디코딩에만 쓴다 (별도 의존성을 늘리지 않으려고)
    from PyQt5 import QtGui, QtWidgets
    app = QtWidgets.QApplication(sys.argv[:1])   # 이미지 플러그인 로드에 필요

    scale = meters_per_pixel(args.lat, args.zoom)
    center_x, center_y = tile_float(args.lat, args.lon, args.zoom)
    half_tiles = args.span / 2 / scale / TILE_PX
    x0 = int(math.floor(center_x - half_tiles))
    x1 = int(math.ceil(center_x + half_tiles))
    y0 = int(math.floor(center_y - half_tiles))
    y1 = int(math.ceil(center_y + half_tiles))
    nx, ny = x1 - x0, y1 - y0
    width, height = nx * TILE_PX, ny * TILE_PX

    print(f"[{args.name}] {label}")
    print(f"중심 {args.lat}, {args.lon}   줌 {args.zoom}   {scale:.3f} m/px")
    print(f"타일 {nx}x{ny} = {nx * ny}장 -> {width}x{height}px "
          f"({width * scale:.0f}m x {height * scale:.0f}m)")

    longest = max(width, height)
    ram_mb = width * height * 4 / 1024 / 1024
    if longest > MAX_MOSAIC_PX:
        print(f"\n중단: 모자이크 {longest}px (RGBA {ram_mb:.0f}MB)가 상한 {MAX_MOSAIC_PX}px를 넘습니다.")
        print(f"  --span을 줄이거나 --zoom {args.zoom - 1} 로 낮추세요.")
        return 1
    if longest > WARN_MOSAIC_PX:
        print(f"  주의: 모자이크 {longest}px, 메모리 RGBA {ram_mb:.0f}MB")

    mosaic = QtGui.QImage(width, height, QtGui.QImage.Format_RGB888)
    mosaic.fill(0)
    painter = QtGui.QPainter(mosaic)

    done = failed = cached = 0
    started = time.time()
    for ty in range(y0, y1):
        for tx in range(x0, x1):
            raw, from_cache = fetch_tile(tx, ty, args.zoom)
            if raw:
                tile = QtGui.QImage.fromData(raw)
                if not tile.isNull():
                    painter.drawImage((tx - x0) * TILE_PX, (ty - y0) * TILE_PX, tile)
                    done += 1
                    cached += from_cache
                else:
                    failed += 1
            else:
                failed += 1
            if (done + failed) % 100 == 0:
                print(f"  {done + failed}/{nx * ny} ...")
            if not from_cache:
                time.sleep(0.05)      # 공공 서비스에 부담 주지 않도록
    painter.end()

    print(f"타일 {done}장 (캐시 {cached} / 신규 {done - cached}), "
          f"실패 {failed}장, {time.time() - started:.1f}s")
    if done == 0:
        print("중단: 한 장도 받지 못했습니다. 인터넷 연결을 확인하세요.")
        return 1

    image_path = os.path.join(args.out, f"{args.name}.png")
    meta_path = os.path.join(args.out, f"{args.name}.json")
    mosaic.save(image_path, "PNG")

    # 모자이크 좌상단이 중심점 기준 몇 미터인지 (동쪽 +x, 북쪽 +y)
    metadata = {
        "name": args.name,
        "label": label,
        "center_lat": args.lat,
        "center_lon": args.lon,
        "zoom": args.zoom,
        "meters_per_pixel": scale,
        "width_px": width,
        "height_px": height,
        "offset_x_m": (x0 * TILE_PX - center_x * TILE_PX) * scale,
        "offset_y_m": (center_y * TILE_PX - y0 * TILE_PX) * scale,
        "tiles": f"{nx}x{ny}",
        "source": "VWorld Satellite (국토교통부)",
        "fetched": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    with open(meta_path, "w", encoding="utf-8") as handle:
        json.dump(metadata, handle, indent=2, ensure_ascii=False)

    size_mb = os.path.getsize(image_path) / 1024 / 1024
    print(f"\n저장 완료:\n  {image_path}  ({size_mb:.1f} MB)\n  {meta_path}")
    print("지상국을 다시 실행하면 3D FLIGHT PATH 바닥에 깔립니다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
