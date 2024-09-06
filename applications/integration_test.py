import unittest
import glob
import os
import sys
import json
import subprocess
import tempfile


class IntegrationTest(unittest.TestCase):
    BINARY_FOLDER = ""
    CONFIG_FILE = ""

    def setUp(self):
        self.working_dir_fp = tempfile.TemporaryDirectory()
        self.working_dir = self.working_dir_fp.name
        print('Running all integration tests in', self.working_dir)

        with open(os.path.join(IntegrationTest.BINARY_FOLDER, IntegrationTest.CONFIG_FILE)) as fp:
            self.main_config = json.load(fp)

    def tearDown(self):
        self.working_dir_fp.cleanup()

    def run_one(self, executable, data_folder, config):

        test_folder = os.path.join(data_folder, config["test_directory"])

        input_tag = config["input_tag"]
        oracle_tag = config["oracle_tag"]
        root_tag = config["input_directory_tag"]

        has_checks = "checks" in config
        checks = [] if not has_checks else config["checks"]

        executable = os.path.join(IntegrationTest.BINARY_FOLDER, "applications", executable)

        for test_file_name in config["tests"]:
            print("Running test", test_file_name)

            test_file = os.path.join(test_folder, test_file_name)

            self.assertTrue(os.path.exists(test_file))

            with open(test_file) as f:
                test_oracle = json.load(f)

            input = test_oracle[input_tag].copy()
            oracle_file = tempfile.NamedTemporaryFile(mode='r')
            input[oracle_tag] = oracle_file.name

            if root_tag in input:
                if not os.path.isabs(input[root_tag]):
                    input[root_tag] = os.path.join(test_folder, input[root_tag])
            else:
                input[root_tag] = test_folder

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
            with self.subTest(msg=key):
                print("Running test for", key)


                file = self.main_config[key]["config_file"]

                with open(file) as fp:
                    config = json.load(fp)

                self.run_one(key, self.main_config[key]["data_folder"], config)


if __name__ == '__main__':
    bin_dir = os.getcwd()
    config_file = "test_config.json"
    if len(sys.argv) > 1:
        bin_dir = os.path.append(bin_dir, sys.argv.pop())

    IntegrationTest.BINARY_FOLDER = bin_dir
    IntegrationTest.CONFIG_FILE = config_file

    unittest.main()