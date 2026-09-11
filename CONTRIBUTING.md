# Contributing to rsubt-mbbluetooth

Thank you for your interest in contributing to rsubt-mbbluetooth! This document explains how to get started.

## Developer Certificate of Origin (DCO)

All contributions require a [Developer Certificate of Origin](https://developercertificate.org/) sign-off. This certifies that you wrote the contribution or have the right to submit it under the project's license.

Add a `Signed-off-by` line to every commit:

```
feat: add support for custom coders

Signed-off-by: Jane Doe <jane.doe@example.com>
```

Git can do this automatically with the `-s` flag:

```bash
git commit -s -m "feat: add support for custom coders"
```

## Pull Request Guidelines

- Keep PRs focused — one feature or fix per PR.
- Sign all commits (`git commit -s`).
- Add tests for new functionality. Test coverage should not decrease.
- Update documentation if your change affects public API.
- Every new source file must include the SPDX license header.
- Use clear, descriptive commit messages following the format below.

## Reporting Issues

- Use [GitHub
Issues](https://github.com/mercedes-benz/rsubt-mbbluetooth/issues) for bug reports and feature requests.
- Include reproduction steps, expected vs. actual behavior, and your environment (Swift version, OS, Xcode version).

## Code of Conduct

This project follows the [Mercedes-Benz Open Source Code of Conduct](CODE_OF_CONDUCT.md). Please read it before participating.
