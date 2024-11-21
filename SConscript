from building import * 
Import('rtconfig')
# get current dir path
cwd = GetCurrentDir()

src = []

# add ina226 src files
src += Glob('ina226.c')
if GetDepend(['PKG_INA226_USING_SENSOR_V1']):
    src += ['ti_ina226_sensor_v1.c']

if GetDepend(['PKG_USING_INA226_EXAMPLE']):
    src += ['example_ina226.c']

# add ina226 inc files
path  = [cwd]
# add src and inc to group 
group = DefineGroup('ina226', src, depend = ['PKG_USING_INA226'], CPPPATH = path)


Return('group')
