import numpy as np
import re
import subprocess
import igl

# basepath = '/home/zhongshi/Workspace/wmtk-freeze/buildr/'
basepath = '/mnt/ff7e01f4-89ad-40d7-a113-112e01c9fd93/jiacheng/wmtk/wildmeshing-toolkit/build/'

def scale(x):
    scale.a = x.min(axis=0)
    y = x-scale.a
    scale.b = y.max()
    y = y/scale.b
    return y


def use_scale(x): return (x-scale.a)/scale.b


def parse_log(regex, loglist):
    rec = re.compile(regex)
    timer = []
    for t, f in loglist:
        with open(f, 'r') as fp:
            matches = ([l for l in fp.readlines()
                        if rec.match(l) is not None])
            assert len(matches) == 1
            time = float(rec.match(matches[0]).group(1))
            timer.append((t, time))
    print(timer)


def write_unit_scale_file(ref, output):
    v, _ = igl.read_triangle_mesh(ref)
    scale(v)

    v1, f1 = igl.read_triangle_mesh(output)
    igl.write_triangle_mesh(output + '.unit.ply', use_scale(v1), f1)


class common_process:
    @classmethod
    def outpath(c):
        return f'{c.base}/out/'

    @classmethod
    def logpath(c):
        return f'{c.base}/log/'

    @classmethod
    def log_info(c):
        parse_log(c.timer_regex, [
                  (t, f'{c.logpath()}/{t}.log') for t in c.threads])

    @classmethod
    def blender_preprocess(c, t=0):
        write_unit_scale_file(c.input, f'{c.outpath()}/{t}.obj')


class fig3_sec(common_process):
    threads = [0, 1, 2, 4, 8, 16, 32]
    base = 'fig3-statue-sec/'
    input = f'{base}/Sapphos_Head.stl'
    timer_regex = r"^.*runtime (.*)$"
    exe = 'app/Release/sec_app'

    @classmethod
    def run(cls):
        for t in cls.threads[::-1]:
            output = f'{cls.outpath()}/{t}.obj'
            params = [basepath + cls.exe, cls.input,
                      output] + f'-j {t}'.split()
            print(' '.join(params))
            with open(f'{cls.logpath()}/{t}.log', 'w') as fp:
                subprocess.run(params, stdout=fp)


class fig4_rem(common_process):
    threads = [0, 1, 2, 4, 8, 16, 32]
    base = 'fig4-hellskull-UniRem/'
    logpath = f'{base}/log/'
    outpath = base + '/out/'
    input = f'{base}/hellSkull_126k.stl'
    timer_regex = r"^.*runtime (.*)$"
    exe = 'app/Release/remeshing_app'

    @classmethod
    def run(cls):
        for t in cls.threads[::-1]:
            output = f'{cls.outpath()}/{t}.obj'
            params = [basepath + cls.exe, cls.input, output] + \
                f'-j {t} -r 0.01 -f 0'.split()
            print(' '.join(params))
            with open(f'{cls.logpath()}/{t}.log', 'w') as fp:
                subprocess.run(params, stdout=fp)


class fig7_secenv(common_process):
    threads = [0, 1, 2, 4, 8, 16, 32]
    base = 'fig7-eins-secenv/'
    input = f'{base}/einstein_big.stl'
    timer_regex = r"^.*runtime (.*)$"
    exe = 'app/Release/sec_app'

    @classmethod
    def run(cls):
        for t in cls.threads[::-1]:
            output = f'{cls.outpath()}/{t}.obj'
            params = [basepath + cls.exe, cls.input, output] + \
                f'-e 1e-3 -j {t} -t 1e-2'.split()
            print(' '.join(params))
            with open(f'{cls.logpath()}/{t}.log', 'w') as fp:
                subprocess.run(params, stdout=fp)


class fig8_rem_env(common_process):
    threads = [0, 1, 2, 4, 8, 16, 32]
    base = 'fig8-headported-UniRemEnv/'
    input = f'{base}/head_-_ported_-_scaled.stl'
    timer_regex = r"^.*runtime (.*)$"
    exe = 'app/Release/remeshing_app'

    @classmethod
    def run(cls):
        for t in cls.threads[::-1]:
            output = f'{cls.outpath()}/{t}.obj'
            params = [basepath + cls.exe, cls.input, output] + \
                f'-e 1e-3 -j {t} -r 0.01 -f 0'.split()
            print(' '.join(params))
            with open(f'{cls.logpath()}/{t}.log', 'w') as fp:
                subprocess.run(params, stdout=fp)

class fig1_fatdragon(common_process):
    threads = [0, 1, 2, 4, 8, 16, 32]
    base = 'fig1-fatdragon-tetwild/'
    input = f'{base}/39507.stl'
    timer_regex = r"^.*total time (.*)s$"
    exe = 'app/tetwild/tetwild'

    @classmethod
    def run(cls):
        for t in cls.threads[::-1]:
            output = f'{cls.outpath()}/{t}.obj'
            params = [basepath + cls.exe, '-i', cls.input, '-o', output] + \
                f'--max-its 10 -j {t}'.split()
            print(' '.join(params))
            with open(f'{cls.logpath()}/{t}.log', 'w') as fp:
                subprocess.run(params, stdout=fp)

class fig12_dragon(common_process):
    threads = [0, 1, 2, 4, 8, 16, 32]
    base = 'fig12-dragon-tetwild'
    input = f'{base}/dragon.off'
    timer_regex = r"^.*total time (.*)s$"
    exe = 'app/tetwild/tetwild'

    @classmethod
    def run(cls):
        for t in cls.threads[::-1]:
            output = f'{cls.outpath()}/{t}.obj'
            params = [basepath + cls.exe, '-i', cls.input, '-o', output] + \
                f'--max-its 10 -j {t}'.split()
            print(' '.join(params))
            print(f'{cls.logpath()}/{t}.log')
            with open(f'{cls.logpath()}/{t}.log', 'w') as fp:
                subprocess.run(params, stdout=fp)

class fig13_fatdragon_skipsim(common_process):
    threads = [0, 1, 2, 4, 8, 16, 32]
    base = 'fig13_fatdragon_skipsim/'
    input = f'{base}/39507.stl'
    timer_regex = r"^.*total time (.*)s$"
    exe = 'app/tetwild/tetwild'

    @classmethod
    def run(cls):
        for t in cls.threads[::-1]:
            output = f'{cls.outpath()}/{t}.obj'
            params = [basepath + cls.exe, '-i', cls.input, '-o', output] + \
                f'--max-its 10 -j {t} --skip-simplify --sample-envelope'.split()
            print(' '.join(params))
            print(f'{cls.logpath()}/{t}.log')
            with open(f'{cls.logpath()}/{t}.log', 'w') as fp:
                subprocess.run(params, stdout=fp)


if __name__ == '__main__':
    # fig7_secenv.run()
    # fig7_secenv.log_info()
    # fig1_fatdragon.run()
    # fig1_fatdragon.log_info()
    fig1_fatdragon.run()
    fig1_fatdragon.log_info()
