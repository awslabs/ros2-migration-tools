import os
from setuptools import setup, find_packages


# Declare your non-python data files:
# Files underneath configuration/ will be copied into the build preserving the
# subdirectory structure if they exist.
data_files = []
for root, dirs, files in os.walk('configuration'):
    data_files.append((os.path.relpath(root, 'configuration'),
                       [os.path.join(root, f) for f in files]))

setup(
    name="B9ROSMigrationTools",
    version="1.0",

    # declare your packages
    packages=find_packages(where="src", exclude=("test",)),
    package_dir={"": "src"},

    # include data files
    data_files=data_files,

    # set up the shebang
    options={
        # make sure the right shebang is set for the scripts - use the
        # environment default Python
        'build_scripts': {
            'executable': '/apollo/sbin/envroot "$ENVROOT/bin/python"',
        },
    },

    # declare your scripts
    # If you want to create any Python executables in bin/, define them here.
    # This is a three-step process:
    #
    # 1. Create the function you want to run on the CLI in src/b9_ros_migration_tools/cli.py
    #    For convenience I usually recommend calling it main()
    #
    # 2. Uncomment this section of the setup.py arguments; this will create
    #    bin/B9ROSMigrationTools (which you can obviously change!) as a script
    #    that will call your main() function, above.
    #
    # entry_points="""\
    # [console_scripts]
    # B9ROSMigrationTools = b9_ros_migration_tools.cli:main
    # """,
    #
    # 3. Add a dependency on BrazilPython-setuptools = default into the
    #    dependencies section of your Config. This is necessary for the script
    #    generated by setuptools to find its function.

    # Setting this value to "true" puts the first version's scripts into
    # $ENVROOT/bin. This is valid for most versionsets, because usually you'll
    # only have the one. If you have a special case where what you really want
    # is to keep a particular version in bin/ then you should change this to a
    # string that identifies that version. Those strings will take the form
    # 'python3.6' or 'jython2.7' etc.
    #
    # Note: You almost certainly don't want to do that.
    root_script_source_version="true",

    # Use the pytest brazilpython runner. Provided by BrazilPython-Pytest.
    test_command='brazilpython_pytest',

    # Use custom sphinx command which adds an index.html that's compatible with
    # code.amazon.com links.
    doc_command='amazon_doc_utils_build_sphinx',
)
