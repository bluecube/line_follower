Import("env")
env.Append(CCFLAGS=["-fsanitize=undefined"], LINKFLAGS=["-fsanitize=undefined"])
