from setuptools import setup, find_packages

setup(name='common',
      version='1.0.0',
      author='luoyifan',
      author_email='luoyifan902008@126.com',
      license="BSD-3-Clause",
      packages=find_packages(),
      description='go2 sport client for ros2 using python',
      python_requires='>=3.8',
      install_requires=[
            "cyclonedds==0.10.2",
            "numpy",

      ],
      )


