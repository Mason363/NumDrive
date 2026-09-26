import zlib
import zopfli.zopfli as _z
def raw_deflate(b):
    c=zlib.compressobj(9, zlib.DEFLATED, -15); return c.compress(b)+c.flush()
def zop(b, it=30):
    z=_z.compress(bytes(b), numiterations=it)
    return z[2:-4]
def check(b):
    d=zop(b)
    assert zlib.decompress(d, -15)==bytes(b)
    return d
