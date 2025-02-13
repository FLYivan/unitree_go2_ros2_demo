from setuptools import setup, find_packages

setup(name='deepseek',
      version='1.0.0',
      author='luoyifan',
      author_email='luoyifan902008@126.com',
      license="BSD-3-Clause",
      packages=find_packages(),
      description='test deepseek local using using python',
      python_requires='>=3.8',
      install_requires=[
            "numpy",
            "opencv-python",
            "openai",
      ],
      )
