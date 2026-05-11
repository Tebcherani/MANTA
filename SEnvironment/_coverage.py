import numpy as np, csv
from pathlib import Path
from datetime import datetime, timezone, timedelta

DATA  = Path(r"Q:\cbofs_data")
EPOCH = datetime(2016, 1, 1, tzinfo=timezone.utc)

files = sorted(f for f in DATA.glob("**/*.npz") if "bathy" not in f.name)
print(f"Files : {len(files)}")
print(f"First : {files[0].name}")
print(f"Last  : {files[-1].name}")

d0 = np.load(files[0]);  t0 = float(d0["ocean_time"][0]);   d0.close()
d1 = np.load(files[-1]); t1 = float(d1["ocean_time"][-1]);  d1.close()
dt0 = EPOCH + timedelta(seconds=t0)
dt1 = EPOCH + timedelta(seconds=t1)
days = (dt1 - dt0).total_seconds() / 86400
print(f"Start : {dt0.strftime('%Y-%m-%d %H:%M UTC')}")
print(f"End   : {dt1.strftime('%Y-%m-%d %H:%M UTC')}")
print(f"Span  : {days:.1f} days  ({len(files)*61} timesteps at 6-min resolution)")

rows = list(csv.DictReader(open(DATA / "station_coords.csv")))
lats = [float(r["lat"]) for r in rows]
lons = [float(r["lon"]) for r in rows]
ns = (max(lats) - min(lats)) * 111.0
ew = (max(lons) - min(lons)) * 111.0 * 0.788
print(f"\nStation coverage ({len(rows)} stations):")
print(f"  lat {min(lats):.3f} to {max(lats):.3f}  (~{ns:.0f} km N-S)")
print(f"  lon {min(lons):.3f} to {max(lons):.3f}  (~{ew:.0f} km E-W)")

b = np.load(DATA / "cbofs_bathy.npz")
m = b["mask_rho"] > 0
blat = b["lat_rho"][m];  blon = b["lon_rho"][m]
bns = (blat.max() - blat.min()) * 111.0
bew = (blon.max() - blon.min()) * 111.0 * 0.788
print(f"\nBathy grid ({m.sum():,} water cells, ~400 m res):")
print(f"  lat {blat.min():.3f} to {blat.max():.3f}  (~{bns:.0f} km N-S)")
print(f"  lon {blon.min():.3f} to {blon.max():.3f}  (~{bew:.0f} km E-W)")
b.close()
