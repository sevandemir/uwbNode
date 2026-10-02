# -*- coding: utf-8 -*-
"""
MDEK1001 (DWM1001 PANS firmware) shell 'lec' çıktısı ayrıştırıcı.

ROS bağımlılığı yoktur. Python 2.7 (ROS Melodic) ve Python 3 (ROS Noetic) uyumludur.

'lec' satır formatı (her anchor 6 alan):
  DIST,<n>,AN0,<id>,<x>,<y>,<z>,<dist>,AN1,<id>,<x>,<y>,<z>,<dist>,...[,POS,<x>,<y>,<z>,<qf>]

Örnek:
  DIST,2,AN0,1151,0.00,0.00,0.00,3.15,AN1,0CA8,5.00,0.00,0.00,2.82,POS,1.54,1.58,0.00,63
"""

import math

ANCHOR_FIELDS = 6   # ANx, id, x, y, z, dist


def _finite(v):
    # math.isfinite Python 2.7'de yok
    return not (math.isinf(v) or math.isnan(v))


class Position(object):
    __slots__ = ('x', 'y', 'z', 'qf')

    def __init__(self, x, y, z, qf):
        self.x, self.y, self.z, self.qf = x, y, z, qf

    def __eq__(self, other):
        return isinstance(other, Position) and \
            (self.x, self.y, self.z, self.qf) == (other.x, other.y, other.z, other.qf)

    def __ne__(self, other):
        return not self == other

    def __repr__(self):
        return 'Position(x={}, y={}, z={}, qf={})'.format(self.x, self.y, self.z, self.qf)


class LecData(object):
    __slots__ = ('distances', 'position')

    def __init__(self, distances=None, position=None):
        self.distances = distances if distances is not None else {}   # anchor_id -> metre
        self.position = position

    def __repr__(self):
        return 'LecData(distances={}, position={})'.format(self.distances, self.position)


def parse_pos(parts):
    """['POS', x, y, z, qf] listesini ayrıştırır. Geçersizse None döner."""
    if len(parts) < 4 or parts[0] != 'POS':
        return None
    try:
        x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
        qf = int(parts[4]) if len(parts) > 4 and parts[4] else 0
    except ValueError:
        return None
    if not all(_finite(v) for v in (x, y, z)):
        return None
    return Position(x, y, z, qf)


def parse_lec_line(line):
    """
    Tek bir 'lec' satırını ayrıştırır.
    DIST veya POS ile başlamayan satırlar (prompt, banner vb.) için None döner.
    """
    # Prompt satırın başına yapışmış olabilir: "dwm> DIST,..."
    starts = [i for i in (line.find('DIST'), line.find('POS')) if i >= 0]
    if not starts:
        return None
    parts = [p.strip() for p in line[min(starts):].split(',')]

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
            if _finite(dist):
                data.distances[anchor_id] = dist
        except ValueError:
            pass
        i += ANCHOR_FIELDS

    if i < len(parts) and parts[i] == 'POS':
        data.position = parse_pos(parts[i:])

    return data
