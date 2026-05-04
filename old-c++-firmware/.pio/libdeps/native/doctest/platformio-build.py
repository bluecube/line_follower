Import("env")
try:
    Import("projenv")
except:
    projenv = env

for env_ in (env, projenv):
    env_.Append(CPPDEFINES=["DOCTEST_CONFIG_COLORS_NONE"])
    cxx_flags = env_.subst("$CXXFLAGS $CCFLAGS")
    if "-std=" not in cxx_flags:
        env_.Append(CXXFLAGS=["-std=c++11"])
