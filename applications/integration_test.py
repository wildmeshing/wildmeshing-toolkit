import unittest

import sys
import os
import json
import subprocess
import tempfile
import argparse

def fix_path(path):
    cwd = os.getcwd()
    return path if os.path.isabs(path) else os.path.join(cwd, path)

    

class IntegrationTest(unittest.TestCase):
    BINARY_FOLDER = fix_path(os.environ['WMTK_BINARY_FOLDER']) if "WMTK_BINARY_FOLDER" in os.environ else None
    CONFIG_FILE = fix_path(os.environ['WMTK_CONFIG_FILE']) if "WMTK_CONFIG_FILE" in os.environ else None
    TEST = None

    def __init__(self, name, config):
        super().__init__()
        self.working_dir_fp = tempfile.TemporaryDirectory()
        self.working_dir = self.working_dir_fp.name

        self.name = name
        self.config = config
        print(f'Loading integration test [{name}] in {self.working_dir}')



    def tearDown(self):
        self.working_dir_fp.cleanup()

    def run_one(self, executable, config_folder, config):
        if "test_directory" in config:
            config_folder = os.path.join(config_folder, config["test_directory"])

        input_tag = config["input_tag"]
        oracle_tag = config["oracle_tag"]
        root_tag = config["input_directory_tag"]

        has_checks = "checks" in config
        checks = [] if not has_checks else config["checks"]

        executable = os.path.join(IntegrationTest.BINARY_FOLDER, executable)

        for test_file_name in config["tests"]:
            print("Running test", test_file_name)

            test_file = os.path.join(config_folder, test_file_name)

            print(f"Test file: {test_file}")
            self.assertTrue(os.path.exists(test_file), f"{test_file} does not exist")

            with open(test_file) as f:
                try:
                    test_oracle = json.load(f)
                except Exception as e:
                    print(f"Caught exception while loading file {test_file}: {e}")
                    raise e

            input = test_oracle[input_tag].copy()
            with tempfile.NamedTemporaryFile(mode='r', delete=False) as oracle_file:
                oracle_file.close()

                input[oracle_tag] = oracle_file.name

                if root_tag in input:
                    if not os.path.isabs(input[root_tag]):
                        input[root_tag] = os.path.join(config_folder, input[root_tag])
                else:
                    input[root_tag] = config_folder

                with tempfile.NamedTemporaryFile(mode='w', delete=False) as input_json:
                    json.dump(input, input_json)
                    input_json.close()


                    cmd = [executable, "-j", input_json.name]
                    res = subprocess.run(cmd, cwd=self.working_dir, capture_output=True)

                if res.returncode != 0:
                    print(f"Error running [{' '.join(cmd)}]")
                    print(res.stderr.decode('utf-8'))
                    print(res.stdout.decode('utf-8'))

                self.assertEqual(res.returncode, 0, f"{res.returncode} != 0")
                with open(oracle_file.name, "r") as fp:
                    result = json.load(fp)

                for check in checks:
                    self.assertEqual(result[check], test_oracle[check], f"{result[check]} != {test_oracle[check]}")

                if len(checks) == 0 and not has_checks:
                    for k in test_oracle:
                        if k == input_tag:
                            continue

                        self.assertTrue(k in result)
                        self.assertEqual(result[k], test_oracle[k], f"{result[k]} != {test_oracle[k]}")


        self.assertTrue(True)


    def runTest(self):
        with self.subTest(msg=self.name):
            print("Running test for", self.name)


            my_config = self.config

            file = my_config["config_file"]

            with open(file) as fp:
                config = json.load(fp)

            data_folder = None if "data_folder" not in my_config else my_config["data_folder"]
            config_folder = data_folder if "config_folder" not in my_config else my_config["config_folder"]
            self.run_one(self.name, config_folder, config)


def make_suite(config_file, single = None):

    suite = unittest.TestSuite()
    with open(config_file) as fp:
        config = json.load(fp)
        for key,value in config.items():
            if key == "skip":
                continue
            if IntegrationTest.TEST and key != IntegrationTest.TEST:
                continue
            if single is None or key == single:
                suite.addTest(IntegrationTest(key,value))
    return suite


            



if __name__ == '__main__':
    parser = argparse.ArgumentParser(
                    prog='WMTK Integration Test',
                    description='Run integration tests for WMTK')

    parser.add_argument('-c', '--test_config', help="Path to the json config file")
    parser.add_argument('-b', '--binary_folder', help="Path to the folder that contains the apps binaries")
    parser.add_argument('-s', '--single', help="Single test to run")
    parser.add_argument('-t', '--test', help="Runs a single test")
    args = parser.parse_args()



    tcin = args.test_config
    bfin = args.binary_folder

    cwd = os.getcwd()
    if tcin:
        config_file = fix_path(tcin)
    else:
        config_file = os.path.join(cwd, "test_config.json")


    if bfin:
        bin_dir = fix_path(bfin)
    else:
        bin_dir = os.path.join(cwd, "applications")




    IntegrationTest.TEST = args.test
    IntegrationTest.BINARY_FOLDER = bin_dir
    IntegrationTest.CONFIG_FILE = config_file

    suite = make_suite(config_file, args.single)

    runner = unittest.TextTestRunner()
    runner.run(suite)
    #unittest.main()

