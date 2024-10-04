#!/usr/bin/env python

import sys

hdr="""
#pragma once
#include <nlohmann/json.hpp>



namespace wmtk::applications::{application_name} {{
    const static nlohmann::json spec;
}}
"""

cpp="""
#include "{file_prefix}"



namespace wmtk::applications::{application_name} {{
    const static nlohmann::json spec = R(
    "
    {json_data}
    "
    )_json;
}}
"""

def __main__():
    application_name, spec_path,file_prefix= sys.argv[1:]



    with open(spec_path) as spec_file:
        json_data = ''.join(spec_file.readlines())
        dat = {
                "application_name": application_name,
                "json_data": json_data,
                "file_prefix":file_prefix
                }

        with open(hdr_path,"w") as hdr_file:
            hdr_path.write(hdr.format(**dat))
        print(cpp.format(**dat))
        pass


if __name__ == "__main__":
    __main__()
