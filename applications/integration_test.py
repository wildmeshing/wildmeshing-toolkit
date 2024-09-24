import unittest

import sys
import os
import json
import subprocess
import tempfile
import argparse


class IntegrationTest(unittest.TestCase):
    BINARY_FOLDER = None
    CONFIG_FILE = None
    TEST = None

    def setUp(self):
        self.working_dir_fp = tempfile.TemporaryDirectory()
        self.working_dir = self.working_dir_fp.name
        print('Running all integration tests in', self.working_dir)

        with open(IntegrationTest.CONFIG_FILE) as fp:
            self.main_config = json.load(fp)

    def tearDown(self):
        self.working_dir_fp.cleanup()

    def run_one(self, executable, config_folder, data_folder, config):
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
            self.assertTrue(os.path.exists(test_file))

            with open(test_file) as f:
                test_oracle = json.load(f)

            input = test_oracle[input_tag].copy()
            oracle_file = tempfile.NamedTemporaryFile(mode='r')
            input[oracle_tag] = oracle_file.name

            if root_tag in input:
                if not os.path.isabs(input[root_tag]):
                    input[root_tag] = os.path.join(config_folder, input[root_tag])
            else:
                input[root_tag] = config_folder

            input_json = tempfile.NamedTemporaryFile(mode='w')
            json.dump(input, input_json)
            input_json.flush()

            res = subprocess.run([executable, "-j", input_json.name], cwd=self.working_dir)

            self.assertEqual(res.returncode, 0)
            result = json.load(oracle_file)

            for check in checks:
                self.assertTrue(result[check], test_oracle[check])

            if len(checks) == 0 and not has_checks:
                for k in test_oracle:
                    if k == input_tag:
                        continue

                    self.assertTrue(k in result)
                    self.assertEqual(result[k], test_oracle[k])

            oracle_file.close()
            input_json.close()


        self.assertTrue(True)

    def test_all(self):
        for key in self.main_config:
            if key == "skip":
                continue
            if IntegrationTest.TEST and key != IntegrationTest.TEST:
                continue

            with self.subTest(msg=key):
                print("Running test for", key)


                my_config = self.main_config[key]

                file = my_config["config_file"]

                with open(file) as fp:
                    config = json.load(fp)

                data_folder = None if "data_folder" not in my_config else my_config["data_folder"]
                config_folder = data_folder if "config_folder" not in my_config else my_config["config_folder"]
                self.run_one(key, config_folder, data_folder, config)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
                    prog='WMTK Integration Test',
                    description='Run integration tests for WMTK')

    parser.add_argument('-c', '--test_config', help="Path to the json config file")
    parser.add_argument('-b', '--binary_folder', help="Path to the folder that contains the apps binaries")
    parser.add_argument('-t', '--test', help="Runs a single test")
    args = parser.parse_args()



    tcin = args.test_config
    bfin = args.binary_folder

    cwd = os.getcwd()
    if tcin:
        config_file = str(tcin if os.path.isabs(tcin) else os.path.join(cwd, tcin))
        sys.argv.pop()
        sys.argv.pop()
    else:
        config_file = os.path.join(cwd, "test_config.json")


    if bfin:
        bin_dir = bfin if os.path.isabs(bfin) else os.path.join(cwd, bfin)
        sys.argv.pop()
        sys.argv.pop()
    else:
        bin_dir = os.path.join(cwd, "applications")


    if args.test:
        sys.argv.pop()
        sys.argv.pop()


    IntegrationTest.TEST = args.test
    IntegrationTest.BINARY_FOLDER = bin_dir
    IntegrationTest.CONFIG_FILE = config_file

    unittest.main()
