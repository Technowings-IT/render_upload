# Flutter App Release Guide

This guide explains how to build and release your AMR Fleet Management app for different platforms.

## Prerequisites

1. Ensure Flutter is properly installed and configured
2. Run `flutter doctor` to verify all dependencies
3. Ensure your app builds successfully in debug mode first

## 1. Android (APK) Release

### Build Release APK
```bash
cd frontend
flutter build apk --release
```

### Build App Bundle (Recommended for Google Play Store)
```bash
flutter build appbundle --release
```

### Output Location
- APK: `build/app/outputs/flutter-apk/app-release.apk`
- App Bundle: `build/app/outputs/bundle/release/app-release.aab`

### File Size
- APK: ~26.8MB (optimized with tree-shaking)

## 2. Linux Release

### Build Linux Executable
```bash
cd frontend
flutter build linux --release
```

### Output Location
- Executable: `build/linux/x64/release/bundle/`
- Main executable: `build/linux/x64/release/bundle/frontend`

### Distribution
The entire `bundle` folder needs to be distributed as it contains:
- The main executable (`frontend`)
- Required libraries
- Assets and resources

### Running on Linux
```bash
cd build/linux/x64/release/bundle/
./frontend
```

## 3. Windows Release

### Build Windows Executable
```bash
cd frontend
flutter build windows --release
```

### Output Location
- Executable: `build/windows/x64/runner/Release/`
- Main executable: `build/windows/x64/runner/Release/frontend.exe`

### Distribution
The entire `Release` folder needs to be distributed as it contains:
- The main executable (`frontend.exe`)
- Required DLLs
- Assets and resources

**Note:** Windows builds require Windows development environment or cross-compilation setup.

## 4. Web Release

### Build Web Application
```bash
cd frontend
flutter build web --release
```

### Output Location
- Web files: `build/web/`
- Main file: `build/web/index.html`

### File Size
- Optimized with tree-shaking (fonts reduced by 98%+)

### Deployment
Deploy the entire `build/web/` folder to your web server. The app can be accessed through `index.html`.

## 5. iOS Release (if needed)

### Build iOS App
```bash
cd frontend
flutter build ios --release
```

**Note:** iOS builds require macOS and Xcode.

## Build Optimization

### Tree Shaking
All builds automatically include tree-shaking to reduce file sizes:
- MaterialIcons: Reduced by 98.7% (1.6MB → 21KB)
- CupertinoIcons: Reduced by 99.4% (257KB → 1.5KB)

### Disable Tree Shaking (if needed)
```bash
flutter build [platform] --no-tree-shake-icons
```

## Release Checklist

### Before Building
- [ ] Test app thoroughly in debug mode
- [ ] Update version numbers in `pubspec.yaml`
- [ ] Update app icons and splash screens
- [ ] Verify all permissions are correctly configured
- [ ] Test on target devices/platforms

### For Android
- [ ] Configure signing key for release builds
- [ ] Update `android/app/build.gradle` with correct version codes
- [ ] Test installation from APK

### For Linux/Windows
- [ ] Test executable on clean systems
- [ ] Verify all dependencies are included
- [ ] Create installer packages if needed

### For Web
- [ ] Test in different browsers
- [ ] Verify responsive design
- [ ] Configure web server properly

## Distribution Methods

### Android
1. **Direct APK Distribution**: Share the APK file directly
2. **Google Play Store**: Upload the App Bundle (AAB)
3. **Alternative Stores**: F-Droid, Amazon Appstore, etc.

### Linux
1. **Direct Distribution**: Share the bundle folder as ZIP/TAR
2. **AppImage**: Create portable application
3. **Package Managers**: Create .deb, .rpm, or Snap packages
4. **Flatpak**: Universal package format

### Windows
1. **Direct Distribution**: Share the Release folder as ZIP
2. **Installer**: Create MSI or NSIS installer
3. **Microsoft Store**: Package as MSIX

### Web
1. **Self-hosted**: Deploy to your own web server
2. **Static Hosting**: Netlify, Vercel, GitHub Pages
3. **CDN**: CloudFlare, AWS CloudFront

## Troubleshooting

### Common Issues
1. **Missing Dependencies**: Ensure all platform-specific dependencies are installed
2. **Signing Issues**: Configure proper certificates for release builds
3. **Plugin Compatibility**: Verify all plugins support target platforms
4. **Performance**: Test on low-end devices for Android

### Platform-Specific Issues
- **Android**: Check ProGuard/R8 configuration
- **Linux**: Verify system library compatibility
- **Windows**: Ensure Visual C++ redistributables
- **Web**: Check browser compatibility and CORS settings

## File Locations Summary

```
build/
├── app/outputs/flutter-apk/app-release.apk     # Android APK
├── app/outputs/bundle/release/app-release.aab  # Android App Bundle
├── linux/x64/release/bundle/                   # Linux executable
├── windows/x64/runner/Release/                 # Windows executable
└── web/                                        # Web application
```

## Security Considerations

1. **Code Obfuscation**: Use `--obfuscate` flag for sensitive apps
2. **API Keys**: Ensure sensitive data is not exposed in builds
3. **Signing**: Use proper signing certificates for all platforms
4. **Updates**: Implement secure update mechanisms

## Next Steps

After building, consider:
1. Setting up CI/CD pipelines for automated builds
2. Implementing crash reporting and analytics
3. Setting up beta testing programs
4. Creating update mechanisms for deployed apps
