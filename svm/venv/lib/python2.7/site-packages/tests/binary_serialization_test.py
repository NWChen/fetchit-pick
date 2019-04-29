import bqplot
import numpy as np

def test_binary_serialize_1d(figure):
    x = np.arange(10, dtype=np.float64)
    y = (x**2).astype(np.int32)
    scatter = bqplot.Scatter(x=x, y=y)

    state = scatter.get_state()
    assert state['x']['dtype'] == 'float64'
    assert state['y']['dtype'] == 'int32'
    
    assert state['x']['buffer'] == memoryview(x)
    assert state['y']['buffer'] == memoryview(y)

    assert state['x']['shape'] == (10,)
    assert state['y']['shape'] == (10,)

    scatter2 = bqplot.Scatter()
    scatter2.set_state(state)

    assert scatter.x.dtype == np.float64
    assert scatter.y.dtype == np.int32

    assert scatter.x.shape == (10,)
    assert scatter.y.shape == (10,)

    assert scatter2.x.tolist() == x.tolist()
    assert scatter2.y.tolist() == y.tolist()

def test_binary_serialize_datetime():
    x = np.arange('2005-02-25', '2005-03', dtype='datetime64[D]')
    x_ms = np.array([1109289600000, 1109376000000, 1109462400000, 1109548800000], dtype=np.int64)
    scatter = bqplot.Scatter(x=x)

    state = scatter.get_state()
    assert state['x']['dtype'] == 'float64'
    assert np.array(state['x']['buffer'], dtype=np.float64).astype(np.int64).tolist() == x_ms.tolist()


    # currently a roundtrip does not converse the datetime64 type
    scatter2 = bqplot.Scatter()
    scatter2.set_state(state)

    assert scatter2.x.dtype == np.float64
    assert scatter2.x.tolist() == x_ms.tolist()

