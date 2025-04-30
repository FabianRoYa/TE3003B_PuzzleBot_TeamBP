import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tony/PerlasNegras/install/blackpearls_nav2_puzzlebot'
