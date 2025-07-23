#!/usr/bin/env python3
"""
Enhanced JSON to PGM+YAML Map Converter
Converts JSON map data to ROS2 compatible PGM and YAML format
Supports multiple input formats and enhanced error handling
"""

import json
import numpy as np
import yaml
import argparse
import os
import sys
from PIL import Image
import logging
from typing import Dict, Any, Tuple, Optional

class EnhancedMapConverter:
    def __init__(self):
        self.default_resolution = 0.05  # 5cm per pixel
        self.default_origin = [-10.0, -10.0, 0.0]  # Default origin
        self.occupied_threshold = 0.65
        self.free_threshold = 0.196
        
        # Setup logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s'
        )
        self.logger = logging.getLogger(__name__)
        
    def convert_json_to_pgm_yaml(self, json_file_path: str, output_dir: Optional[str] = None, 
                                map_name: Optional[str] = None) -> Dict[str, Any]:
        """
        Convert JSON map data to PGM and YAML files
        
        Args:
            json_file_path (str): Path to the JSON map file
            output_dir (str): Output directory for PGM and YAML files
            map_name (str): Name for the output files
            
        Returns:
            Dict containing conversion results
        """
        try:
            self.logger.info(f"Starting conversion of {json_file_path}")
            
            # Read JSON data
            with open(json_file_path, 'r') as f:
                map_data = json.load(f)
            
            # Set defaults if not provided
            if output_dir is None:
                output_dir = os.path.dirname(json_file_path)
            
            if map_name is None:
                map_name = os.path.splitext(os.path.basename(json_file_path))[0]
            
            # Ensure output directory exists
            os.makedirs(output_dir, exist_ok=True)
            
            # Convert map data
            pgm_data, yaml_data = self._process_map_data(map_data, map_name)
            
            # Write PGM file
            pgm_path = os.path.join(output_dir, f"{map_name}.pgm")
            self._write_pgm_file(pgm_data, pgm_path)
            
            # Write YAML file
            yaml_path = os.path.join(output_dir, f"{map_name}.yaml")
            self._write_yaml_file(yaml_data, yaml_path, f"{map_name}.pgm")
            
            self.logger.info(f"Successfully converted {json_file_path}")
            self.logger.info(f"PGM file: {pgm_path}")
            self.logger.info(f"YAML file: {yaml_path}")
            
            return {
                'pgm_path': pgm_path,
                'yaml_path': yaml_path,
                'success': True,
                'map_info': yaml_data,
                'dimensions': {
                    'width': pgm_data.shape[1],
                    'height': pgm_data.shape[0]
                }
            }
            
        except Exception as e:
            error_msg = f"Error converting map: {str(e)}"
            self.logger.error(error_msg)
            return {
                'success': False,
                'error': error_msg
            }
    
    def _process_map_data(self, map_data: Dict[str, Any], map_name: str) -> Tuple[np.ndarray, Dict[str, Any]]:
        """Process JSON map data and create PGM and YAML data"""
        
        self.logger.info("Processing map data...")
        
        # Handle different JSON structures
        actual_data = self._extract_map_data(map_data)
        
        # Get map dimensions and data
        width, height = self._get_map_dimensions(actual_data)
        occupancy_data = self._get_occupancy_data(actual_data, width, height)
        
        # Get metadata
        resolution = self._get_resolution(actual_data)
        origin = self._get_origin(actual_data)
        
        self.logger.info(f"Map dimensions: {width}x{height}, resolution: {resolution}")
        
        # Convert occupancy data to numpy array
        occupancy_array = np.array(occupancy_data, dtype=np.float32)
        
        # Reshape to 2D array if needed
        if len(occupancy_array.shape) == 1:
            occupancy_array = occupancy_array.reshape((height, width))
        
        # Validate dimensions
        if occupancy_array.shape != (height, width):
            self.logger.warning(f"Reshaping array from {occupancy_array.shape} to ({height}, {width})")
            # Resize or pad as needed
            occupancy_array = self._resize_occupancy_array(occupancy_array, height, width)
        
        # Convert to PGM format (0-255)
        pgm_array = self._convert_to_pgm_values(occupancy_array)
        
        # Create YAML data
        yaml_data = {
            'image': f"{map_name}.pgm",
            'resolution': resolution,
            'origin': origin,
            'negate': 0,
            'occupied_thresh': self.occupied_threshold,
            'free_thresh': self.free_threshold,
            'mode': 'trinary'
        }
        
        return pgm_array, yaml_data
    
    def _extract_map_data(self, map_data: Dict[str, Any]) -> Dict[str, Any]:
        """Extract actual map data from various JSON structures"""
        
        # Handle nested structures
        if 'mapData' in map_data:
            return map_data['mapData']
        elif 'data' in map_data and isinstance(map_data['data'], dict):
            return map_data['data']
        else:
            return map_data
    
    def _get_map_dimensions(self, actual_data: Dict[str, Any]) -> Tuple[int, int]:
        """Extract map dimensions from data"""
        
        # Try various field names for width and height
        width = None
        height = None
        
        # Check info section first
        if 'info' in actual_data:
            info = actual_data['info']
            width = info.get('width')
            height = info.get('height')
        
        # Direct width/height fields
        if width is None:
            width = actual_data.get('width')
        if height is None:
            height = actual_data.get('height')
        
        # Default values if not found
        if width is None:
            width = 384
            self.logger.warning(f"Width not found, using default: {width}")
        if height is None:
            height = 384
            self.logger.warning(f"Height not found, using default: {height}")
        
        return int(width), int(height)
    
    def _get_occupancy_data(self, actual_data: Dict[str, Any], width: int, height: int) -> list:
        """Extract occupancy data from various possible locations"""
        
        occupancy_data = None
        
        # Try different field names for occupancy data
        data_fields = ['occupancyData', 'data', 'occupancy_grid', 'grid_data', 'map_data']
        
        for field in data_fields:
            if field in actual_data and actual_data[field]:
                occupancy_data = actual_data[field]
                self.logger.info(f"Found occupancy data in field: {field}")
                break
        
        # If no data found, create empty map
        if occupancy_data is None:
            self.logger.warning("No occupancy data found, creating empty map")
            occupancy_data = [-1] * (width * height)  # Unknown cells
        
        return occupancy_data
    
    def _get_resolution(self, actual_data: Dict[str, Any]) -> float:
        """Extract resolution from data"""
        
        resolution = None
        
        # Check info section first
        if 'info' in actual_data:
            resolution = actual_data['info'].get('resolution')
        
        # Direct resolution field
        if resolution is None:
            resolution = actual_data.get('resolution')
        
        # Default resolution
        if resolution is None:
            resolution = self.default_resolution
            self.logger.warning(f"Resolution not found, using default: {resolution}")
        
        return float(resolution)
    
    def _get_origin(self, actual_data: Dict[str, Any]) -> list:
        """Extract origin from data"""
        
        origin = None
        
        # Check info section first
        if 'info' in actual_data:
            info = actual_data['info']
            if 'origin' in info:
                origin_data = info['origin']
                if isinstance(origin_data, dict):
                    # Handle position/orientation structure
                    if 'position' in origin_data:
                        pos = origin_data['position']
                        origin = [pos.get('x', 0.0), pos.get('y', 0.0), pos.get('z', 0.0)]
                    else:
                        origin = [origin_data.get('x', 0.0), origin_data.get('y', 0.0), origin_data.get('z', 0.0)]
                elif isinstance(origin_data, list):
                    origin = origin_data[:3]  # Take first 3 elements
        
        # Direct origin field
        if origin is None:
            origin = actual_data.get('origin')
        
        # Default origin
        if origin is None:
            origin = self.default_origin[:]
            self.logger.warning(f"Origin not found, using default: {origin}")
        
        # Ensure we have 3 elements
        while len(origin) < 3:
            origin.append(0.0)
        
        return [float(x) for x in origin[:3]]
    
    def _resize_occupancy_array(self, array: np.ndarray, target_height: int, target_width: int) -> np.ndarray:
        """Resize occupancy array to target dimensions"""
        
        current_height, current_width = array.shape
        
        if current_height == target_height and current_width == target_width:
            return array
        
        self.logger.info(f"Resizing array from {current_height}x{current_width} to {target_height}x{target_width}")
        
        # Create new array filled with unknown values (-1)
        new_array = np.full((target_height, target_width), -1, dtype=np.float32)
        
        # Copy existing data to the center or top-left
        copy_height = min(current_height, target_height)
        copy_width = min(current_width, target_width)
        
        new_array[:copy_height, :copy_width] = array[:copy_height, :copy_width]
        
        return new_array
    
    def _convert_to_pgm_values(self, occupancy_array: np.ndarray) -> np.ndarray:
        """Convert occupancy values to PGM format (0-255)"""
        
        self.logger.info(f"Converting occupancy data with shape: {occupancy_array.shape}")
        self.logger.info(f"Data range: [{occupancy_array.min()}, {occupancy_array.max()}]")
        
        # Handle different input formats
        if occupancy_array.dtype == np.bool_:
            # Boolean array: True = occupied, False = free
            pgm_array = np.where(occupancy_array, 0, 255)  # Occupied = 0 (black), Free = 255 (white)
        elif occupancy_array.max() <= 1.0 and occupancy_array.min() >= -1.0:
            # Probability values (-1 = unknown, 0.0 = free, 1.0 = occupied)
            pgm_array = np.where(occupancy_array < 0, 128,  # Unknown = gray
                                np.where(occupancy_array > 0.5, 0,  # Occupied = black
                                        255))  # Free = white
        else:
            # ROS standard values: -1 = unknown, 0 = free, 100 = occupied
            pgm_array = np.where(occupancy_array < 0, 128,  # Unknown = gray
                                np.where(occupancy_array > 50, 0,  # Occupied = black
                                        255))  # Free = white
        
        return pgm_array.astype(np.uint8)
    
    def _write_pgm_file(self, pgm_data: np.ndarray, file_path: str):
        """Write PGM file in P5 format"""
        
        height, width = pgm_data.shape
        
        self.logger.info(f"Writing PGM file: {file_path} ({width}x{height})")
        
        with open(file_path, 'wb') as f:
            # Write PGM header
            f.write(b'P5\n')
            f.write(f'# Created by Enhanced ROS2 Map Converter\n'.encode())
            f.write(f'# Original data shape: {pgm_data.shape}\n'.encode())
            f.write(f'{width} {height}\n'.encode())
            f.write(b'255\n')
            
            # Write binary data
            # Flip vertically for ROS coordinate system (origin at bottom-left)
            flipped_data = np.flipud(pgm_data)
            f.write(flipped_data.tobytes())
    
    def _write_yaml_file(self, yaml_data: Dict[str, Any], file_path: str, pgm_filename: str):
        """Write YAML metadata file"""
        
        yaml_data['image'] = pgm_filename
        
        self.logger.info(f"Writing YAML file: {file_path}")
        
        with open(file_path, 'w') as f:
            yaml.dump(yaml_data, f, default_flow_style=False, sort_keys=False)
    
    def validate_map_files(self, pgm_path: str, yaml_path: str) -> Dict[str, Any]:
        """Validate generated map files"""
        
        validation_result = {
            'valid': True,
            'errors': [],
            'warnings': [],
            'file_sizes': {}
        }
        
        try:
            # Check PGM file
            if os.path.exists(pgm_path):
                pgm_size = os.path.getsize(pgm_path)
                validation_result['file_sizes']['pgm'] = pgm_size
                self.logger.info(f"PGM file size: {pgm_size} bytes")
                
                # Try to read PGM header
                with open(pgm_path, 'rb') as f:
                    header = f.read(20).decode('ascii', errors='ignore')
                    if not header.startswith('P5'):
                        validation_result['errors'].append("Invalid PGM header")
                        validation_result['valid'] = False
            else:
                validation_result['errors'].append("PGM file not found")
                validation_result['valid'] = False
            
            # Check YAML file
            if os.path.exists(yaml_path):
                yaml_size = os.path.getsize(yaml_path)
                validation_result['file_sizes']['yaml'] = yaml_size
                self.logger.info(f"YAML file size: {yaml_size} bytes")
                
                # Try to parse YAML
                with open(yaml_path, 'r') as f:
                    yaml_content = yaml.safe_load(f)
                    
                # Check required fields
                required_fields = ['image', 'resolution', 'origin', 'occupied_thresh', 'free_thresh']
                for field in required_fields:
                    if field not in yaml_content:
                        validation_result['errors'].append(f"Missing required YAML field: {field}")
                        validation_result['valid'] = False
                        
                validation_result['yaml_content'] = yaml_content
            else:
                validation_result['errors'].append("YAML file not found")
                validation_result['valid'] = False
                
        except Exception as e:
            validation_result['errors'].append(f"Validation error: {str(e)}")
            validation_result['valid'] = False
        
        return validation_result
    
    def convert_from_dict(self, map_data_dict: Dict[str, Any], output_dir: str, map_name: str) -> Dict[str, Any]:
        """
        Convert map data directly from a dictionary (for Node.js integration)
        
        Args:
            map_data_dict (dict): Map data as dictionary
            output_dir (str): Output directory
            map_name (str): Map name
        
        Returns:
            dict: Result with success status and file paths
        """
        try:
            self.logger.info(f"Converting map data dictionary to {map_name}")
            
            # Ensure output directory exists
            os.makedirs(output_dir, exist_ok=True)
            
            # Process map data
            pgm_data, yaml_data = self._process_map_data(map_data_dict, map_name)
            
            # Write files
            pgm_path = os.path.join(output_dir, f"{map_name}.pgm")
            yaml_path = os.path.join(output_dir, f"{map_name}.yaml")
            
            self._write_pgm_file(pgm_data, pgm_path)
            self._write_yaml_file(yaml_data, yaml_path, f"{map_name}.pgm")
            
            # Validate files
            validation = self.validate_map_files(pgm_path, yaml_path)
            
            result = {
                'success': True,
                'pgm_path': pgm_path,
                'yaml_path': yaml_path,
                'map_info': yaml_data,
                'validation': validation,
                'dimensions': {
                    'width': pgm_data.shape[1],
                    'height': pgm_data.shape[0]
                }
            }
            
            self.logger.info(f"Successfully converted dictionary to {map_name}")
            return result
            
        except Exception as e:
            error_msg = f"Dictionary conversion error: {str(e)}"
            self.logger.error(error_msg)
            return {
                'success': False,
                'error': error_msg
            }
    
    def get_map_info(self, yaml_path: str) -> Dict[str, Any]:
        """Get information about an existing map from its YAML file"""
        
        try:
            with open(yaml_path, 'r') as f:
                yaml_content = yaml.safe_load(f)
            
            # Get corresponding PGM file info
            pgm_path = yaml_path.replace('.yaml', '.pgm')
            pgm_info = {}
            
            if os.path.exists(pgm_path):
                pgm_info['exists'] = True
                pgm_info['size'] = os.path.getsize(pgm_path)
                
                # Try to get PGM dimensions
                try:
                    with Image.open(pgm_path) as img:
                        pgm_info['width'], pgm_info['height'] = img.size
                except Exception as e:
                    self.logger.warning(f"Could not read PGM dimensions: {e}")
            else:
                pgm_info['exists'] = False
            
            return {
                'success': True,
                'yaml_content': yaml_content,
                'pgm_info': pgm_info,
                'files': {
                    'yaml': yaml_path,
                    'pgm': pgm_path
                }
            }
            
        except Exception as e:
            return {
                'success': False,
                'error': str(e)
            }

def main():
    parser = argparse.ArgumentParser(description='Enhanced JSON to PGM+YAML Map Converter')
    parser.add_argument('input_file', help='Input JSON map file')
    parser.add_argument('-o', '--output-dir', help='Output directory')
    parser.add_argument('-n', '--name', help='Output map name')
    parser.add_argument('-v', '--validate', action='store_true', help='Validate output files')
    parser.add_argument('--log-level', choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'], 
                       default='INFO', help='Set logging level')
    
    args = parser.parse_args()
    
    # Set logging level
    logging.getLogger().setLevel(getattr(logging, args.log_level))
    
    if not os.path.exists(args.input_file):
        print(f"Error: Input file '{args.input_file}' not found", file=sys.stderr)
        sys.exit(1)
    
    converter = EnhancedMapConverter()
    result = converter.convert_json_to_pgm_yaml(
        args.input_file,
        args.output_dir,
        args.name
    )
    
    if result['success']:
        print("✅ Conversion completed successfully!")
        print(f"PGM file: {result['pgm_path']}")
        print(f"YAML file: {result['yaml_path']}")
        
        if args.validate:
            print("\n🔍 Validating files...")
            validation = converter.validate_map_files(result['pgm_path'], result['yaml_path'])
            
            if validation['valid']:
                print("✅ Validation passed!")
                print(f"PGM size: {validation['file_sizes'].get('pgm', 0)} bytes")
                print(f"YAML size: {validation['file_sizes'].get('yaml', 0)} bytes")
            else:
                print("❌ Validation failed!")
                for error in validation['errors']:
                    print(f"  - {error}")
                sys.exit(1)
        
        sys.exit(0)
    else:
        print(f"❌ Conversion failed: {result['error']}", file=sys.stderr)
        sys.exit(1)

# For use as a module from Node.js
def convert_map_from_node(json_data: Dict[str, Any], output_dir: str, map_name: str) -> Dict[str, Any]:
    """
    Function to be called from Node.js
    
    Args:
        json_data (dict): Map data as dictionary
        output_dir (str): Output directory
        map_name (str): Map name
    
    Returns:
        dict: Result with success status and file paths
    """
    converter = EnhancedMapConverter()
    return converter.convert_from_dict(json_data, output_dir, map_name)

if __name__ == '__main__':
    main()