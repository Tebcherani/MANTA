import sys, numpy as np
sys.path.insert(0, ".")
from SEnvironment.cbofs_viewer import load_archive
from pathlib import Path

D = load_archive(Path(r"Q:\cbofs_data"))
print("U shape :", D["U"].shape)
print("MAG range:", round(float(np.nanmin(D["MAG"])), 4),
      "-", round(float(np.nanmax(D["MAG"])), 4), "m/s")
print("Z range  :", round(float(np.nanmin(D["Z"])), 4),
      "-", round(float(np.nanmax(D["Z"])), 4), "m")
print("NaN in U :", int(np.isnan(D["U"]).sum()), "/", D["U"].size)
print("Load OK")
