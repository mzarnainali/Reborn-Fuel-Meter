Import("env")
env.AddCustomTarget(
    "uploadnobuild", 
    None, 
    'pio run -e %s -t nobuild -t upload --disable-auto-clean' % 
        env["PIOENV"], 
    title="Uploads without building"
)