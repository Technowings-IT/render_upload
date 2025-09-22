#  PGM Map Editor - GIMP-like Map Editing Guide

##  Overview
The PGM Map Editor provides a professional GIMP-like interface for visually editing ROS2 navigation maps. Edit maps with precision using familiar tools, then deploy directly to your Raspberry Pi robots.

##  Quick Start

### 1. Access the Editor
1. Open the AMR Fleet Management app
2. Navigate to **Maps** tab
3. Select **PGM Editor** tab (4th tab)
4. You'll see the GIMP-like interface with:
   -  **Toolbar** (top)
   -  **Tool Panel** (left)
   - ️ **Canvas** (center)
   - ️ **Properties Panel** (right)

### 2. Load a Map
```
 Load Map → Select from:
• Existing JSON maps (converts to PGM)
• Previously saved PGM files
• Create new blank map
```

## ️ Editing Tools

### Primary Tools
| Tool | Icon | Function | Usage |
|------|------|----------|-------|
| **Brush** | ️ | Paint free space | Click/drag to mark areas as navigable |
| **Eraser** |  | Remove obstacles | Click/drag to clear obstacles |
| **Line** |  | Draw straight paths | Click start → drag → click end |
| **Rectangle** | ⬜ | Create room outlines | Click corner → drag → release |
| **Flood Fill** |  | Fill enclosed areas | Click to fill connected region |

### Advanced Tools
- **Zoom** : Mouse wheel or toolbar buttons
- **Pan** : Right-click drag or middle mouse
- **Undo/Redo** ↩️↪️: Ctrl+Z / Ctrl+Y
- **Grid Toggle** : Show/hide alignment grid
- **Layers** : Manage map layers

##  Brush Settings

### Brush Size
```
Small:  ●  (1-3 pixels) - Fine detail work
Medium: ●● (4-8 pixels) - General editing  
Large:  ●●● (9-15 pixels) - Quick coverage
```

### Map Values
- **Free Space** (255): White - robots can navigate
- **Obstacles** (0): Black - walls, furniture, no-go zones
- **Unknown** (128): Gray - unexplored areas

##  Workflow Example

### Editing a Warehouse Map
```
1.  Load → Select "warehouse_scan.json"
2. ️ Brush → Paint clear pathways
3.  Eraser → Remove false obstacles from doorways  
4. ⬜ Rectangle → Define no-go zones around equipment
5.  Line → Mark preferred robot routes
6.  Flood Fill → Mark large open areas
7.  Save → "warehouse_edited.pgm"
8.  Deploy → Send to Robot "AMR-001"
```

##  Save & Deploy Options

### Save Formats
- **PGM + YAML**: Standard ROS2 navigation format
- **JSON**: AMR Fleet Management format
- **PNG**: Visual preview for documentation

### Deployment
```
 Deploy to Robot:
1. Select target robot from dropdown
2. Choose deployment options:
   • Replace current map
   • Add as new map
   • Backup existing first
3. Click "Deploy to Pi"
4. Monitor deployment status
```

##  Keyboard Shortcuts

| Action | Shortcut | Description |
|--------|----------|-------------|
| Brush | `B` | Select brush tool |
| Eraser | `E` | Select eraser tool |
| Line | `L` | Select line tool |
| Rectangle | `R` | Select rectangle tool |
| Fill | `F` | Select flood fill |
| Zoom In | `+` or `Ctrl+=` | Increase zoom |
| Zoom Out | `-` or `Ctrl+-` | Decrease zoom |
| Fit to Screen | `0` | Reset zoom to fit |
| Undo | `Ctrl+Z` | Undo last action |
| Redo | `Ctrl+Y` | Redo action |
| Save | `Ctrl+S` | Quick save |
| Grid Toggle | `G` | Show/hide grid |

##  Advanced Editing Tips

### Precision Editing
- Use **Grid** for pixel-perfect alignment
- **Zoom in** (200-400%) for detail work
- **Small brush** for fine corrections
- **Line tool** for straight walls

### Large Area Editing  
- **Large brush** for quick coverage
- **Flood fill** for rooms and open spaces
- **Rectangle tool** for regular shapes
- **Medium zoom** (100-150%) for overview

### Professional Workflow
```
1.  Zoom out → Plan overall changes
2.  Mark problem areas → Note what needs fixing
3. ️ Rough editing → Block out major changes
4.  Zoom in → Refine details
5.  Quality check → Verify navigation paths
6.  Save versions → Keep backup copies
```

##  Troubleshooting

### Common Issues
| Problem | Solution |
|---------|----------|
| **Map too dark** | Increase brightness in properties |
| **Brush not working** | Check tool selection and brush size |
| **Undo not working** | Ensure you're in edit mode |
| **Deploy fails** | Verify robot connection and permissions |
| **Large file slow** | Try reducing map resolution |

### Performance Tips
- **Close other apps** for smoother editing
- **Save frequently** to prevent data loss
- **Use smaller brush** for better responsiveness
- **Reduce zoom** if editing becomes laggy

##  Map Quality Guidelines

### Navigation Requirements
 **Good Map Characteristics:**
- Clear pathways (white) between rooms
- Accurate wall boundaries (black)
- Consistent obstacle marking
- Proper door representations
- No isolated free spaces

 **Avoid:**
- Thin passages (< robot width)
- Floating obstacles  
- Inconsistent wall thickness
- Missing door openings
- Excessive unknown areas

### Validation Checklist
- [ ] All doorways are properly marked as free space
- [ ] Robot can navigate between all required areas
- [ ] No-go zones are clearly marked as obstacles
- [ ] Map resolution is appropriate for robot size
- [ ] Origin point is correctly positioned

##  Deployment Best Practices

### Pre-Deployment
1. **Test locally** with path planning
2. **Validate dimensions** match real environment  
3. **Check robot clearance** in narrow areas
4. **Backup existing map** before replacement

### Post-Deployment
1. **Monitor robot behavior** during first runs
2. **Check navigation accuracy** in edited areas
3. **Fine-tune** if needed based on performance
4. **Document changes** for team reference

##  Support

Need help? Check:
-  **Documentation**: Full API reference  
-  **Issues**: Report bugs and feature requests
-  **Community**: Join our Discord for help
-  **Support**: team@AMR-fleet.com

---

**Happy Mapping! ️**

*Transform your robot navigation with professional map editing tools.*
