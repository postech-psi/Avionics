"""Compare the native FP32 kernel with PyTorch using both 112.txt replay grids."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

import joblib
import numpy as np
import pandas as pd
import torch

from export_gru import FEATURES


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('executable', type=Path)
    parser.add_argument('psintelligence', type=Path)
    args = parser.parse_args()
    root = args.psintelligence.resolve()
    sys.path.insert(0, str(root))
    from apogee_models import GRUApogeeModel, raw_apogee
    from apogee_pipeline import slice_history

    experiments = root / 'data/experiments'
    bundle = experiments / 'apogee-v2-m7-ukf-burst-aug/w100_seed42'
    config = json.loads((bundle / 'run.json').read_text(encoding='utf-8'))
    scaler = joblib.load(bundle / 'scaler.pkl')
    model = GRUApogeeModel(hidden_dim=config['hidden_dim'], num_layers=config['num_layers'])
    model.load_state_dict(torch.load(bundle / 'model.pth', map_location='cpu', weights_only=True))
    model.eval()
    torch.set_num_threads(2)
    trace = pd.read_csv(experiments / 'apogee-v2-m7-ukf-gap/112_validation_trace.csv')
    logged = pd.read_csv(experiments / 'apogee-v2-m7-ukf-burst-aug/112_comparison.csv')
    for mode, group in trace.groupby('mode'):
        window = config['window']
        indices = np.flatnonzero(group.usable_prediction & group.t_since_accel_trigger.ge(0))
        assert len(indices) > 0
        x = slice_history(group[list(FEATURES)].to_numpy(dtype=np.float32), indices, window)
        h, v = group.iloc[indices][['altitude_rel', 'velocity_z']].to_numpy(dtype=np.float32).T
        rows = np.column_stack((h, v, x.reshape(len(x), -1)))
        result = subprocess.run([str(args.executable.resolve())], input=''.join(
            ' '.join(f'{a:.9g}' for a in row) + '\n' for row in rows), encoding='utf-8',
            check=True, capture_output=True)
        cpp = np.array([list(map(float, line.split())) for line in result.stdout.splitlines()])
        with torch.inference_mode():
            xt = torch.from_numpy(scaler.transform(x.reshape(-1, len(FEATURES))).reshape(x.shape))
            r = model(xt)[0]
            altitude = raw_apogee(torch.from_numpy(h), torch.from_numpy(v), r)
        assert cpp.shape == (len(indices), 2)
        np.testing.assert_allclose(cpp[:, 0], r.numpy(), rtol=0, atol=3e-6)
        np.testing.assert_allclose(cpp[:, 1], altitude.numpy(), rtol=0, atol=.002)
        reference = logged.loc[(logged['mode'] == mode) & (logged.model == 'augmented') &
                               (logged.seed == 42), 'final_m'].item()
        np.testing.assert_allclose(cpp[-1, 1], reference, rtol=0, atol=.002)
        print(f'{mode}: {len(indices)} windows, max |Δr|={max(abs(cpp[:,0] - r.numpy())):.3g}, '
              f'max |Δh|={max(abs(cpp[:,1] - altitude.numpy())):.3g} m, final={cpp[-1,1]:.5f} m')


if __name__ == '__main__':
    main()
