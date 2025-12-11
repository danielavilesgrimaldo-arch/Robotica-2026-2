import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/robousr/Robotica-2026-2/proyecto_final_viejo/install/final_project'
