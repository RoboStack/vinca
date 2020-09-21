import networkx as nx
import yaml
import glob
import sys, os
import textwrap
from collections import OrderedDict
from yaml import CLoader as Loader, CDumper as Dumper

# def setup_yaml():
#   """ https://stackoverflow.com/a/8661021 """
#   represent_dict_order = lambda self, data:  self.represent_mapping('tag:yaml.org,2002:map', data.items())
#   yaml.add_representer(OrderedDict, represent_dict_order)
# setup_yaml()

# the stages

"""
# use the official gcc image, based on debian
# can use verions as well, like gcc:5.2
# see https://hub.docker.com/_/gcc/
image: ubuntu:20.04

build:
  stage: build
  script:
    - echo "Hello"
"""


# def str_presenter(dumper, data):
#   if len(data.splitlines()) > 1:  # check for multiline string
#     return dumper.represent_scalar('tag:yaml.org,2002:str', data, style='|')
#   return dumper.represent_scalar('tag:yaml.org,2002:str', data)

# yaml.add_representer(str, str_presenter)

azure_linux_script = """export CI=azure
export GIT_BRANCH=$BUILD_SOURCEBRANCHNAME
export FEEDSTOCK_NAME=$(basename ${BUILD_REPOSITORY_NAME})
.scripts/run_docker_build.sh"""

azure_osx_script = """export CI=azure
export GIT_BRANCH=$BUILD_SOURCEBRANCHNAME
export FEEDSTOCK_NAME=$(basename ${BUILD_REPOSITORY_NAME})
.scripts/build_osx.sh"""

azure_win_script = """
set "CI=azure"
call activate base

@rem git clone https://github.com/thesnakepit/boa
@rem cd boa
@rem pip install -e .
@rem cd ..

conda config --append channels defaults
conda config --add channels conda-forge
conda config --add channels robostack
conda config --set channel_priority strict

@rem conda info
@rem conda config --show-sources

conda remove --force m2-git

@rem conda list --show-channel-urls

cp recipes/%CURRENT_BUILD_PKG_NAME%.yaml ./recipe.yaml

boa build .

@rem conda.exe build "recipe" -m .ci_support/%CONFIG%.yaml
"""

def main():
	metas = []

	for f in glob.glob(os.path.join(sys.argv[1], "*.yaml")):
		print(f)
		with open(f) as fi:
			metas.append(yaml.load(fi.read(), Loader=Loader))

	requirements = {}

	for pkg in metas:
		requirements[pkg['package']['name']] = pkg['requirements']['host'] + pkg['requirements']['run']

	print(requirements)

	G = nx.DiGraph()
	for pkg, reqs in requirements.items():
		G.add_node(pkg)
		for r in reqs:
			if r.startswith('ros-'):
				G.add_edge(pkg, r)

	# import matplotlib.pyplot as plt
	# nx.draw(G, with_labels=True, font_weight='bold')
	# plt.show()

	tg = list(reversed(list(nx.topological_sort(G))))
	print(tg)

	stages = []
	current_stage = []
	for pkg in tg:
		for r in requirements[pkg]:
			if r in current_stage:
				stages.append(current_stage)
				current_stage = []
		current_stage.append(pkg)

	stages.append(current_stage)

	print(stages)

	azure_template = {
		# 'image': 'condaforge/linux-anvil-cos7-x86_64'
	}

	azure_template = {
		'pool': {
		    'vmImage': 'ubuntu-16.04'
		}
	}

	azure_stages = []

	stage_names = []
	for i, s in enumerate(stages):
		stage_name = f'stage_{i}'
		stage = {
			'stage': stage_name,
			'jobs': []
		}
		stage_names.append(stage_name)


		for pkg in s:
			pkg_jobname = pkg.replace('-', '_')
			stage['jobs'].append({
				'job': pkg_jobname,
				'steps':
				[{
					# 'script': '''.scripts/build_linux.sh''',
					'script': azure_linux_script,
					'env': {
						'ANACONDA_TOKEN': '$(ANACONDA_TOKEN)',
						'CURRENT_BUILD_PKG_NAME': pkg
					},
					'displayName': f'Build {pkg}'
				}]
			})
		azure_stages.append(stage)

	# azure_template['trigger'] = ['experimental']
	azure_template['trigger'] = ['']
	azure_template['pr'] = 'none'
	azure_template['stages'] = azure_stages

	with open('linux.yml', 'w') as fo:
		fo.write(yaml.dump(azure_template, Dumper=Dumper, sort_keys=False))

	azure_template = {
		'pool': {
		    'vmImage': 'macOS-10.15'
		}
	}

	azure_stages = []

	stage_names = []
	for i, s in enumerate(stages):
		stage_name = f'stage_{i}'
		stage = {
			'stage': stage_name,
			'jobs': []
		}
		stage_names.append(stage_name)


		for pkg in s:
			pkg_jobname = pkg.replace('-', '_')
			stage['jobs'].append({
				'job': pkg_jobname,
				'steps':
				[{
					# 'script': '''.scripts/build_linux.sh''',
					'script': azure_osx_script,
					'env': {
						'ANACONDA_TOKEN': '$(ANACONDA_TOKEN)',
						'CURRENT_BUILD_PKG_NAME': pkg
					},
					'displayName': f'Build {pkg}'
				}]
			})
		azure_stages.append(stage)

	# azure_template['trigger'] = ['experimental']
	azure_template['trigger'] = ['']
	azure_template['pr'] = 'none'
	azure_template['stages'] = azure_stages

	with open('osx.yml', 'w') as fo:
		fo.write(yaml.dump(azure_template, Dumper=Dumper, sort_keys=False))

	# windows
	azure_template = {
		'pool': {
		    'vmImage': 'vs2017-win2016'
		}
	}

	azure_stages = []

	stage_names = []
	for i, s in enumerate(stages):
		stage_name = f'stage_{i}'
		stage = {
			'stage': stage_name,
			'jobs': []
		}
		stage_names.append(stage_name)

		for pkg in s:
			pkg_jobname = pkg.replace('-', '_')
			stage['jobs'].append({
				'job': pkg_jobname,
				'variables': {
					'CONDA_BLD_PATH': 'C:\\\\bld\\\\'
				},
				'steps':
				[
				{
					'task': 'CondaEnvironment@1',
					'inputs': {
						'packageSpecs': 'python=3.6 dataclasses conda-build conda conda-forge::conda-forge-ci-setup=3 pip boa quetz-client',
				        'installOptions': "-c conda-forge/label/boa_dev -c conda-forge",
				        'updateConda': True
				    },
				    'displayName': 'Install conda-build, boa and activate environment'
				},
				# {
				# 	'script': "rmdir C:\\cygwin /s /q",
				# 	'displayName': 'Remove cygwin to make git work',
				# 	'continueOnError': True
				# },
				{
					'script': textwrap.dedent("""
						set "CI=azure"
						call activate base
						run_conda_forge_build_setup"""),
				    'displayName': 'conda-forge build setup'
				},
				{
					'script': azure_win_script,
					'env': {
						'ANACONDA_TOKEN': '$(ANACONDA_TOKEN)',
						'CURRENT_BUILD_PKG_NAME': pkg,
						'PYTHONUNBUFFERED': 1
					},
					'displayName': f'Build {pkg}'
				}]
			})
		azure_stages.append(stage)

	azure_template['trigger'] = ['experimental']
	azure_template['pr'] = 'none'
	azure_template['stages'] = azure_stages

	with open('win.yml', 'w') as fo:
		fo.write(yaml.dump(azure_template, Dumper=Dumper, sort_keys=False))