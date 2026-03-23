from pathlib import Path

from setuptools import find_packages, setup

package_name = 'simulator'


def package_files(*directories: str) -> list[tuple[str, list[str]]]:
    data_files: dict[str, list[str]] = {}
    root = Path('.')

    # Never package runtime-generated files or caches
    _EXCLUDE_DIRS = {'generated', '__pycache__', '.pytest_cache', '.vscode', '.claude'}

    for directory in directories:
        for path in (root / directory).rglob('*'):
            if not path.is_file():
                continue
            if any(part in _EXCLUDE_DIRS for part in path.parts):
                continue
            install_dir = f'share/{package_name}/{path.parent.as_posix()}'
            data_files.setdefault(install_dir, []).append(path.as_posix())

    return sorted(data_files.items())

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *package_files('fs_bringup', 'fs_track'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rohith',
    maintainer_email='rohithm8988@gmail.com',
    description='Formula Student track generation, launch helpers, and TUI scaffolding',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'generate_track = fs_track_generator.cli:main',
            'fs_tui = fs_tui.panel:main',
        ],
    },
)
