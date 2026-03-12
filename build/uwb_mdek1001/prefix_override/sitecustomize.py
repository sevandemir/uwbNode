import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/muhammed-servan/PycharmProjects/uwbNode/install/uwb_mdek1001'
