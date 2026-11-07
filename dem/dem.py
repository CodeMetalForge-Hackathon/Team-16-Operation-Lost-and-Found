import rasterio

DEM_PATH = "data/wmnf_demo_dem.tif"

def load_dem(path=DEM_PATH):
    src = rasterio.open(path)
    dem = src.read(1, masked=True)  # elevation in meters
    return src, dem

if __name__ == "__main__":
    src, dem = load_dem()
    print("CRS:", src.crs)                       # should say EPSG:26919
    print("Size:", dem.shape, "pixels")
    print("Elevation range: %.0f to %.0f m" % (dem.min(), dem.max()))

    # Look up the elevation at the center of the tile
    x = (src.bounds.left + src.bounds.right) / 2
    y = (src.bounds.bottom + src.bounds.top) / 2
    row, col = src.index(x, y)
    print("Center (UTM %.0f, %.0f) elevation: %.0f m" % (x, y, dem[row, col]))