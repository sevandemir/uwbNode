"""
MDEK1001 (DWM1001 PANS firmware) shell 'lec' çıktısı ayrıştırıcı.

ROS bağımlılığı yoktur; cihaz olmadan test edilebilir.

'lec' satır formatı (her anchor 6 alan):
  DIST,<n>,AN0,<id>,<x>,<y>,<z>,<dist>,AN1,<id>,<x>,<y>,<z>,<dist>,...[,POS,<x>,<y>,<z>,<qf>]

Örnek:
  DIST,2,AN0,1151,0.00,0.00,0.00,3.15,AN1,0CA8,5.00,0.00,0.00,2.82,POS,1.54,1.58,0.00,63
"""

import math
from dataclasses import dataclass, field
from typing import Optional

ANCHOR_FIELDS = 6   # ANx, id, x, y, z, dist


@dataclass
class Position:
    x: float
    y: float
    z: float
    qf: int


@dataclass
class LecData:
    distances: dict[str, float] = field(default_factory=dict)   # anchor_id -> metre
    position: Optional[Position] = None


def parse_pos(parts: list[str]) -> Optional[Position]:
    """['POS', x, y, z, qf] listesini ayrıştırır. Geçersizse None döner."""
    if len(parts) < 4 or parts[0] != 'POS':
        return None
    try:
        x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
        qf = int(parts[4]) if len(parts) > 4 and parts[4] else 0
    except ValueError:
        return None
    if not all(math.isfinite(v) for v in (x, y, z)):
        return None
    return Position(x, y, z, qf)


def parse_lec_line(line: str) -> Optional[LecData]:
    """
    Tek bir 'lec' satırını ayrıştırır.
    DIST veya POS ile başlamayan satırlar (prompt, banner vb.) için None döner.
    """
    # Prompt satırın başına yapışmış olabilir: "dwm> DIST,..."
    start = min((i for i in (line.find('DIST'), line.find('POS')) if i >= 0), default=-1)
    if start < 0:
        return None
    parts = [p.strip() for p in line[start:].split(',')]

    if parts[0] == 'POS':
        pos = parse_pos(parts)
        return LecData(position=pos) if pos else None

    if parts[0] != 'DIST':
        return None

    data = LecData()
    i = 2   # parts[1] = anchor sayısı, ilk anchor bloğu buradan başlar
    while i + ANCHOR_FIELDS <= len(parts) and parts[i].startswith('AN'):
        anchor_id = parts[i + 1]
        try:
            dist = float(parts[i + 5])
            if math.isfinite(dist):
                data.distances[anchor_id] = dist
        except ValueError:
            pass
        i += ANCHOR_FIELDS

    if i < len(parts) and parts[i] == 'POS':
        data.position = parse_pos(parts[i:])

    return data
