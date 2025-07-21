#!/Users/leyi/anaconda3/envs/pyvista_env/bin/python
"""
Bijective Map App Web GUI
A Python web interface for the bijective_map_app CLI tool.
"""

import os
import sys
import subprocess
import webbrowser
from pathlib import Path
from http.server import HTTPServer, BaseHTTPRequestHandler
import urllib.parse
import json
import tempfile
import shutil

class BijectiveMapHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(self.get_main_page().encode())
        elif self.path.startswith('/browse'):
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            
            # Parse query parameters
            query = urllib.parse.urlparse(self.path).query
            params = urllib.parse.parse_qs(query)
            
            # Get directory path from query parameter or use data directory as default
            default_path = str(Path.cwd() / 'data') if (Path.cwd() / 'data').exists() else str(Path.cwd())
            dir_path = params.get('path', [default_path])[0]
            file_type = params.get('type', ['file'])[0]  # 'file' or 'directory'
            
            try:
                current_dir = Path(dir_path)
                if not current_dir.exists() or not current_dir.is_dir():
                    current_dir = Path.cwd()
                
                files = []
                directories = []
                
                # Add parent directory option (unless at root)
                if current_dir.parent != current_dir:
                    directories.append({
                        'name': '..',
                        'path': str(current_dir.parent.absolute()),
                        'type': 'dir',
                        'is_parent': True
                    })
                
                # List directory contents
                for item in sorted(current_dir.iterdir(), key=lambda x: (not x.is_dir(), x.name.lower())):
                    if item.is_dir():
                        directories.append({
                            'name': item.name,
                            'path': str(item.absolute()),
                            'type': 'dir',
                            'is_parent': False
                        })
                    elif item.is_file():
                        # Show all files, but mark supported ones
                        is_supported = item.suffix.lower() in ['.obj', '.msh', '.off', '.vtu', '.png', '.jpg', '.jpeg', '.tiff', '.bmp']
                        files.append({
                            'name': item.name,
                            'path': str(item.absolute()),
                            'type': 'file',
                            'size': item.stat().st_size,
                            'extension': item.suffix.lower(),
                            'is_supported': is_supported
                        })
                
                result = {
                    'current_path': str(current_dir.absolute()),
                    'directories': directories,
                    'files': files,
                    'file_type': file_type
                }
                
            except Exception as e:
                result = {
                    'error': str(e),
                    'current_path': str(Path.cwd()),
                    'directories': [],
                    'files': []
                }
            
            self.wfile.write(json.dumps(result).encode())
        else:
            self.send_response(404)
            self.end_headers()
    
    def do_POST(self):
        if self.path == '/run':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data.decode('utf-8'))
            
            result = self.run_bijective_map_app(data)
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(result).encode())
    
    def run_bijective_map_app(self, params):
        """Execute the bijective_map_app with given parameters"""
        try:
            # Build command
            cmd = ['./build/bijective_map/bijective_map_app']
            
            # Required parameters
            if not params.get('input_file'):
                return {'success': False, 'error': 'Input file is required'}
            if not params.get('output_file'):
                return {'success': False, 'error': 'Output file is required'}
            if not params.get('logs_dir'):
                return {'success': False, 'error': 'Logs directory is required'}
            
            cmd.extend(['-i', params['input_file']])
            cmd.extend(['-o', params['output_file']])
            cmd.extend(['-l', params['logs_dir']])
            
            if params.get('app_mode'):
                cmd.extend(['-a', params['app_mode']])
            
            # Optional parameters for texture transfer
            if params.get('input_obj_file'):
                cmd.extend(['--input_obj', params['input_obj_file']])
            if params.get('input_texture_file'):
                cmd.extend(['--input_texture', params['input_texture_file']])
            if params.get('width_out'):
                cmd.extend(['--width_out', str(params['width_out'])])
            if params.get('height_out'):
                cmd.extend(['--height_out', str(params['height_out'])])
            
            # Parameters for iso_lines mode
            if params.get('app_mode') == 'iso_lines':
                if params.get('N'):
                    cmd.extend(['--N', str(params['N'])])
                # Handle parallel flag (checkbox returns 'on' when checked, None when unchecked)
                if params.get('do_parallel') != 'on':  # If not checked, add --no_parallel
                    cmd.append('--no_parallel')
            
            # Execute command
            print(f"Executing: {' '.join(cmd)}")
            result = subprocess.run(cmd, capture_output=True, text=True, cwd=Path.cwd())
            
            return {
                'success': result.returncode == 0,
                'returncode': result.returncode,
                'stdout': result.stdout,
                'stderr': result.stderr,
                'command': ' '.join(cmd)
            }
            
        except Exception as e:
            return {'success': False, 'error': str(e)}
    
    def get_main_page(self):
        return """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Bijective Map App GUI</title>
    <style>
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            max-width: 1200px;
            margin: 0 auto;
            padding: 20px;
            background-color: #f5f5f5;
        }
        .container {
            background: white;
            border-radius: 8px;
            padding: 30px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
        }
        h1 {
            color: #333;
            border-bottom: 2px solid #007acc;
            padding-bottom: 10px;
        }
        .form-group {
            margin-bottom: 20px;
        }
        label {
            display: block;
            margin-bottom: 5px;
            font-weight: 500;
            color: #555;
        }
        input, select, textarea {
            width: 100%;
            padding: 10px;
            border: 1px solid #ddd;
            border-radius: 4px;
            font-size: 14px;
            box-sizing: border-box;
        }
        input[type="number"] {
            width: 150px;
        }
        .checkbox-group {
            display: flex;
            align-items: center;
            margin-bottom: 10px;
        }
        .checkbox-group input {
            width: auto;
            margin-right: 10px;
        }
        button {
            background: #007acc;
            color: white;
            border: none;
            padding: 12px 24px;
            border-radius: 4px;
            cursor: pointer;
            font-size: 16px;
            margin-right: 10px;
        }
        button:hover {
            background: #005a9f;
        }
        button:disabled {
            background: #ccc;
            cursor: not-allowed;
        }
        .file-input-group {
            display: flex;
            align-items: center;
        }
        .file-input-group input {
            flex: 1;
            margin-right: 10px;
        }
        .browse-btn {
            background: #28a745;
            padding: 10px 16px;
            font-size: 14px;
        }
        .browse-btn:hover {
            background: #218838;
        }
        .app-modes {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 10px;
        }
        .mode-option {
            padding: 10px;
            border: 2px solid #ddd;
            border-radius: 4px;
            cursor: pointer;
            text-align: center;
            transition: all 0.2s;
        }
        .mode-option:hover {
            border-color: #007acc;
            background: #f0f8ff;
        }
        .mode-option.selected {
            border-color: #007acc;
            background: #007acc;
            color: white;
        }
        .optional-section {
            background: #f8f9fa;
            padding: 20px;
            border-radius: 4px;
            margin-top: 20px;
        }
        .result-section {
            margin-top: 30px;
            padding: 20px;
            background: #f8f9fa;
            border-radius: 4px;
            display: none;
        }
        .success {
            border-left: 4px solid #28a745;
            background: #d4edda;
        }
        .error {
            border-left: 4px solid #dc3545;
            background: #f8d7da;
        }
        .command-preview {
            background: #2d3748;
            color: #e2e8f0;
            padding: 15px;
            border-radius: 4px;
            font-family: 'Monaco', 'Menlo', monospace;
            font-size: 12px;
            margin-top: 15px;
            overflow-x: auto;
        }
        .log-output {
            background: #1a202c;
            color: #e2e8f0;
            padding: 15px;
            border-radius: 4px;
            font-family: 'Monaco', 'Menlo', monospace;
            font-size: 12px;
            white-space: pre-wrap;
            max-height: 400px;
            overflow-y: auto;
            margin-top: 10px;
        }
        .required {
            color: #dc3545;
        }
        
        /* File Browser Modal */
        .modal {
            display: none;
            position: fixed;
            z-index: 1000;
            left: 0;
            top: 0;
            width: 100%;
            height: 100%;
            background-color: rgba(0,0,0,0.5);
        }
        
        .modal-content {
            background-color: #fefefe;
            margin: 5% auto;
            padding: 0;
            border-radius: 8px;
            width: 80%;
            max-width: 800px;
            height: 80%;
            display: flex;
            flex-direction: column;
        }
        
        .modal-header {
            padding: 20px;
            border-bottom: 1px solid #ddd;
            display: flex;
            justify-content: space-between;
            align-items: center;
        }
        
        .modal-body {
            flex: 1;
            padding: 20px;
            overflow: hidden;
            display: flex;
            flex-direction: column;
        }
        
        .path-bar {
            background: #f8f9fa;
            padding: 10px;
            border-radius: 4px;
            margin-bottom: 15px;
            font-family: monospace;
            font-size: 14px;
            overflow-x: auto;
        }
        
        .file-list {
            flex: 1;
            border: 1px solid #ddd;
            border-radius: 4px;
            overflow-y: auto;
        }
        
        .file-item {
            display: flex;
            align-items: center;
            padding: 12px;
            border-bottom: 1px solid #eee;
            cursor: pointer;
            transition: background-color 0.2s;
        }
        
        .file-item:hover {
            background-color: #f8f9fa;
        }
        
        .file-item.selected {
            background-color: #e3f2fd;
            border-color: #2196f3;
        }
        
        .file-icon {
            margin-right: 12px;
            font-size: 18px;
            width: 20px;
            text-align: center;
        }
        
        .file-info {
            flex: 1;
        }
        
        .file-name {
            font-weight: 500;
            margin-bottom: 2px;
        }
        
        .file-details {
            font-size: 12px;
            color: #666;
        }
        
        .file-item.unsupported {
            opacity: 0.6;
        }
        
        .file-item.unsupported .file-name {
            color: #999;
        }
        
        .modal-footer {
            padding: 20px;
            border-top: 1px solid #ddd;
            display: flex;
            justify-content: space-between;
        }
        
        .close {
            color: #aaa;
            font-size: 28px;
            font-weight: bold;
            cursor: pointer;
        }
        
        .close:hover {
            color: #000;
        }
        
        .loading {
            text-align: center;
            padding: 40px;
            color: #666;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🔄 Bijective Map Application GUI</h1>
        
        <form id="bijectiveForm">
            <!-- Required Parameters -->
            <h3>Required Parameters</h3>
            
            <div class="form-group">
                <label for="input_file">Input Mesh File <span class="required">*</span></label>
                <div class="file-input-group">
                    <input type="text" id="input_file" name="input_file" 
                           placeholder="Select input mesh file (.obj, .msh, .off, .vtu)" required>
                    <button type="button" class="browse-btn" onclick="openFilePicker('input_file', false, 'data')">Browse</button>
                </div>
            </div>
            
            <div class="form-group">
                <label for="output_file">Output Mesh File <span class="required">*</span></label>
                <div class="file-input-group">
                    <input type="text" id="output_file" name="output_file" 
                           placeholder="Select output mesh file (.obj, .msh, .off, .vtu)" required>
                    <button type="button" class="browse-btn" onclick="openFilePicker('output_file', false, 'build')">Browse</button>
                </div>
            </div>
            
            <div class="form-group">
                <label for="logs_dir">Logs Directory <span class="required">*</span></label>
                <div class="file-input-group">
                    <input type="text" id="logs_dir" name="logs_dir" 
                           placeholder="Select directory for operation logs" required>
                    <button type="button" class="browse-btn" onclick="openFilePicker('logs_dir', true, 'build')">Browse</button>
                </div>
            </div>
            
            <!-- Application Mode -->
            <div class="form-group">
                <label>Application Mode</label>
                <div class="app-modes">
                    <div class="mode-option selected" data-mode="back">
                        <strong>back</strong><br>
                        <small>Backward tracking (default)</small>
                    </div>
                    <div class="mode-option" data-mode="forward">
                        <strong>forward</strong><br>
                        <small>Forward tracking</small>
                    </div>
                    <div class="mode-option" data-mode="texture">
                        <strong>texture</strong><br>
                        <small>Texture transfer</small>
                    </div>
                    <div class="mode-option" data-mode="render">
                        <strong>render</strong><br>
                        <small>Render index</small>
                    </div>
                    <div class="mode-option" data-mode="back_lines">
                        <strong>back_lines</strong><br>
                        <small>Backward line tracking</small>
                    </div>
                    <div class="mode-option" data-mode="back_r">
                        <strong>back_r</strong><br>
                        <small>Backward tracking (special)</small>
                    </div>
                    <div class="mode-option" data-mode="iso_lines">
                        <strong>iso_lines</strong><br>
                        <small>Iso lines</small>
                    </div>
                    <div class="mode-option" data-mode="check_iso_lines">
                        <strong>check_iso_lines</strong><br>
                        <small>Check iso lines</small>
                    </div>
                </div>
                <input type="hidden" id="app_mode" name="app_mode" value="back">
            </div>
            
            <!-- Optional Parameters -->
            <div class="optional-section" id="textureOptions" style="display: none;">
                <h3>Texture Transfer Options</h3>
                <p><em>Required when using texture mode</em></p>
                
                <div class="form-group">
                    <label for="input_obj_file">Input OBJ File</label>
                    <div class="file-input-group">
                        <input type="text" id="input_obj_file" name="input_obj_file" 
                               placeholder="Select input OBJ file for texture transfer">
                        <button type="button" class="browse-btn" onclick="openFilePicker('input_obj_file', false, 'data')">Browse</button>
                    </div>
                </div>
                
                <div class="form-group">
                    <label for="input_texture_file">Input Texture File</label>
                    <div class="file-input-group">
                        <input type="text" id="input_texture_file" name="input_texture_file" 
                               placeholder="Select texture image file">
                        <button type="button" class="browse-btn" onclick="openFilePicker('input_texture_file', false, 'data')">Browse</button>
                    </div>
                </div>
                
                <div style="display: flex; gap: 20px;">
                    <div class="form-group">
                        <label for="width_out">Output Width</label>
                        <input type="number" id="width_out" name="width_out" value="200" min="1">
                    </div>
                    <div class="form-group">
                        <label for="height_out">Output Height</label>
                        <input type="number" id="height_out" name="height_out" value="200" min="1">
                    </div>
                </div>
            </div>
            
            <div class="optional-section" id="isoLinesOptions" style="display: none;">
                <h3>Iso Lines Options</h3>
                <p><em>Required when using iso_lines mode</em></p>
                
                <div class="form-group">
                    <label for="input_obj_file_iso">Input OBJ File</label>
                    <div class="file-input-group">
                        <input type="text" id="input_obj_file_iso" name="input_obj_file" 
                               placeholder="Select input OBJ file for iso lines">
                        <button type="button" class="browse-btn" onclick="openFilePicker('input_obj_file_iso', false, 'data')">Browse</button>
                    </div>
                </div>
                
                <div style="display: flex; gap: 20px;">
                    <div class="form-group">
                        <label for="N">Number of Isolines (N)</label>
                        <input type="number" id="N" name="N" value="5" min="1" max="100">
                        <small style="color: #666; font-size: 12px;">Number of isolines to generate (default: 5)</small>
                    </div>
                    <div class="form-group">
                        <div class="checkbox-group">
                            <input type="checkbox" id="do_parallel" name="do_parallel" checked>
                            <label for="do_parallel">Enable Parallel Processing</label>
                        </div>
                        <small style="color: #666; font-size: 12px;">Use parallel processing for faster curve tracking (default: enabled)</small>
                    </div>
                </div>
            </div>
            
            <!-- Command Preview -->
            <div class="form-group">
                <label>Command Preview</label>
                <div class="command-preview" id="commandPreview">
                    ./build/bijective_map/bijective_map_app -i [input_file] -o [output_file] -l [logs_dir] -a back
                </div>
            </div>
            
            <!-- Run Button -->
            <button type="submit" id="runBtn">▶️ Run Bijective Map App</button>
            <button type="button" onclick="clearForm()">🗑️ Clear Form</button>
        </form>
        
        <!-- Results -->
        <div class="result-section" id="results">
            <h3>Execution Results</h3>
            <div id="resultContent"></div>
        </div>
    </div>

    <!-- File Browser Modal -->
    <div id="fileBrowserModal" class="modal">
        <div class="modal-content">
            <div class="modal-header">
                <h3 id="modalTitle">Select File</h3>
                <span class="close" onclick="closeFileBrowser()">&times;</span>
            </div>
            <div class="modal-body">
                <div class="path-bar" id="currentPath">/</div>
                <input type="text" id="searchInput" placeholder="🔍 Type to search files..." 
                       style="width: 100%; padding: 10px; margin-bottom: 15px; border: 1px solid #ddd; border-radius: 4px; font-size: 14px;">
                <div class="file-list" id="fileList">
                    <div class="loading">Loading...</div>
                </div>
            </div>
            <div class="modal-footer">
                <div>
                    <button type="button" onclick="goToHome()">🏠 Home</button>
                    <button type="button" onclick="refreshFileList()">🔄 Refresh</button>
                </div>
                <div>
                    <button type="button" onclick="closeFileBrowser()">Cancel</button>
                    <button type="button" id="selectBtn" onclick="selectCurrentFile()" disabled>Select</button>
                </div>
            </div>
        </div>
    </div>

    <script>
        let selectedMode = 'back';
        
        // Mode selection
        document.querySelectorAll('.mode-option').forEach(option => {
            option.addEventListener('click', function() {
                document.querySelectorAll('.mode-option').forEach(o => o.classList.remove('selected'));
                this.classList.add('selected');
                selectedMode = this.dataset.mode;
                document.getElementById('app_mode').value = selectedMode;
                
                // Show/hide optional sections
                document.getElementById('textureOptions').style.display = 
                    selectedMode === 'texture' ? 'block' : 'none';
                document.getElementById('isoLinesOptions').style.display = 
                    selectedMode === 'iso_lines' ? 'block' : 'none';
                
                updateCommandPreview();
            });
        });
        
        // Form submission
        document.getElementById('bijectiveForm').addEventListener('submit', function(e) {
            e.preventDefault();
            runApp();
        });
        
        // Update command preview on input change
        document.querySelectorAll('input').forEach(input => {
            input.addEventListener('input', updateCommandPreview);
        });
        
        // Add specific event listeners for iso_lines options
        document.getElementById('N').addEventListener('input', updateCommandPreview);
        document.getElementById('do_parallel').addEventListener('change', updateCommandPreview);
        
        function updateCommandPreview() {
            const formData = new FormData(document.getElementById('bijectiveForm'));
            const params = Object.fromEntries(formData);
            
            let cmd = './build/bijective_map/bijective_map_app';
            
            if (params.input_file) cmd += ` -i "${params.input_file}"`;
            if (params.output_file) cmd += ` -o "${params.output_file}"`;
            if (params.logs_dir) cmd += ` -l "${params.logs_dir}"`;
            if (params.app_mode) cmd += ` -a ${params.app_mode}`;
            
            if (selectedMode === 'texture' || selectedMode === 'iso_lines') {
                if (params.input_obj_file) cmd += ` --input_obj "${params.input_obj_file}"`;
                if (selectedMode === 'texture') {
                    if (params.input_texture_file) cmd += ` --input_texture "${params.input_texture_file}"`;
                    if (params.width_out) cmd += ` --width_out ${params.width_out}`;
                    if (params.height_out) cmd += ` --height_out ${params.height_out}`;
                } else if (selectedMode === 'iso_lines') {
                    if (params.N) cmd += ` --N ${params.N}`;
                    // Check if parallel processing is disabled
                    const doParallelCheckbox = document.getElementById('do_parallel');
                    if (doParallelCheckbox && !doParallelCheckbox.checked) {
                        cmd += ` --no_parallel`;
                    }
                }
            }
            
            document.getElementById('commandPreview').textContent = cmd;
        }
        
        // File browser variables
        let currentBrowserPath = '';
        let currentTargetInput = '';
        let isSelectingDirectory = false;
        let selectedFilePath = '';
        
        function openFilePicker(inputId, isDirectory = false, defaultDir = '') {
            currentTargetInput = inputId;
            isSelectingDirectory = isDirectory;
            selectedFilePath = '';
            
            // Set modal title
            const modalTitle = document.getElementById('modalTitle');
            modalTitle.textContent = isDirectory ? 'Select Directory' : 'Select File';
            
            // Get current path from input or use default directory
            const input = document.getElementById(inputId);
            let startPath = input.value || '';
            
            // If no current path and default directory provided, use it
            if (!startPath && defaultDir) {
                startPath = defaultDir;
            }
            
            currentBrowserPath = startPath;
            
            // Show modal and load file list
            document.getElementById('fileBrowserModal').style.display = 'block';
            
            // Clear and focus search input
            const searchInput = document.getElementById('searchInput');
            searchInput.value = '';
            setTimeout(() => searchInput.focus(), 100);
            
            loadFileList(currentBrowserPath);
        }
        
        function closeFileBrowser() {
            document.getElementById('fileBrowserModal').style.display = 'none';
            selectedFilePath = '';
            currentTargetInput = '';
        }
        
        async function loadFileList(path = '') {
            const fileList = document.getElementById('fileList');
            const currentPath = document.getElementById('currentPath');
            const selectBtn = document.getElementById('selectBtn');
            
            fileList.innerHTML = '<div class="loading">Loading...</div>';
            selectBtn.disabled = true;
            
            try {
                const url = `/browse?path=${encodeURIComponent(path)}&type=${isSelectingDirectory ? 'directory' : 'file'}`;
                const response = await fetch(url);
                const data = await response.json();
                
                if (data.error) {
                    fileList.innerHTML = `<div class="error">Error: ${data.error}</div>`;
                    return;
                }
                
                currentBrowserPath = data.current_path;
                currentPath.textContent = data.current_path;
                
                let html = '';
                
                // Add directories
                data.directories.forEach(dir => {
                    const icon = dir.is_parent ? '⬆️' : '📁';
                    html += `
                        <div class="file-item" onclick="navigateToDirectory('${dir.path}')" data-type="dir">
                            <div class="file-icon">${icon}</div>
                            <div class="file-info">
                                <div class="file-name">${dir.name}</div>
                                <div class="file-details">Directory</div>
                            </div>
                        </div>
                    `;
                });
                
                // Add files (only if not selecting directory)
                if (!isSelectingDirectory) {
                    data.files.forEach(file => {
                        const icon = getFileIcon(file.extension);
                        const sizeStr = formatFileSize(file.size);
                        const className = file.is_supported ? 'file-item' : 'file-item unsupported';
                        
                        html += `
                            <div class="${className}" onclick="selectFile('${file.path}', '${file.name}')" data-type="file" data-path="${file.path}">
                                <div class="file-icon">${icon}</div>
                                <div class="file-info">
                                    <div class="file-name">${file.name}</div>
                                    <div class="file-details">${sizeStr} • ${file.extension || 'No extension'}</div>
                                </div>
                            </div>
                        `;
                    });
                }
                
                fileList.innerHTML = html || '<div class="loading">No items found</div>';
                
                // Store original file list for search filtering
                window.originalFileData = { directories: data.directories, files: data.files };
                
                // If selecting directory, enable select button for current directory
                if (isSelectingDirectory) {
                    selectBtn.disabled = false;
                    selectedFilePath = currentBrowserPath;
                }
                
            } catch (error) {
                fileList.innerHTML = `<div class="error">Error loading files: ${error.message}</div>`;
            }
        }
        
        function navigateToDirectory(path) {
            loadFileList(path);
        }
        
        function selectFile(path, name) {
            // Remove previous selection
            document.querySelectorAll('.file-item.selected').forEach(item => {
                item.classList.remove('selected');
            });
            
            // Add selection to clicked item
            event.currentTarget.classList.add('selected');
            
            selectedFilePath = path;
            document.getElementById('selectBtn').disabled = false;
        }
        
        function selectCurrentFile() {
            if (selectedFilePath) {
                const input = document.getElementById(currentTargetInput);
                input.value = selectedFilePath;
                updateCommandPreview();
                closeFileBrowser();
            }
        }
        
        function goToHome() {
            loadFileList('');
        }
        
        function refreshFileList() {
            loadFileList(currentBrowserPath);
        }
        
        function getFileIcon(extension) {
            const icons = {
                '.obj': '🔺',
                '.msh': '🔷',
                '.off': '⚫',
                '.vtu': '📊',
                '.png': '🖼️',
                '.jpg': '🖼️',
                '.jpeg': '🖼️',
                '.tiff': '🖼️',
                '.bmp': '🖼️'
            };
            return icons[extension?.toLowerCase()] || '📄';
        }
        
        function formatFileSize(bytes) {
            if (bytes === 0) return '0 B';
            const k = 1024;
            const sizes = ['B', 'KB', 'MB', 'GB'];
            const i = Math.floor(Math.log(bytes) / Math.log(k));
            return parseFloat((bytes / Math.pow(k, i)).toFixed(1)) + ' ' + sizes[i];
        }
        
        async function runApp() {
            const runBtn = document.getElementById('runBtn');
            const results = document.getElementById('results');
            const resultContent = document.getElementById('resultContent');
            
            // Clear previous results
            results.style.display = 'none';
            resultContent.innerHTML = '';
            results.className = 'result-section'; // Reset classes
            
            // Disable button and show loading
            runBtn.disabled = true;
            runBtn.textContent = '⏳ Running...';
            
            // Collect form data
            const formData = new FormData(document.getElementById('bijectiveForm'));
            const params = Object.fromEntries(formData);
            
            // Handle iso_lines mode specific parameters
            if (selectedMode === 'iso_lines') {
                if (document.getElementById('input_obj_file_iso').value) {
                    params.input_obj_file = document.getElementById('input_obj_file_iso').value;
                }
                if (document.getElementById('N').value) {
                    params.N = document.getElementById('N').value;
                }
                // For checkbox, we need to check if it's checked
                const doParallelCheckbox = document.getElementById('do_parallel');
                if (doParallelCheckbox.checked) {
                    params.do_parallel = 'on';
                }
            }
            
            try {
                const response = await fetch('/run', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify(params)
                });
                
                const result = await response.json();
                
                // Show results
                results.style.display = 'block';
                results.className = 'result-section ' + (result.success ? 'success' : 'error');
                
                let html = `
                    <h4>${result.success ? '✅ Success' : '❌ Error'}</h4>
                    <p><strong>Command:</strong> <code>${result.command || 'N/A'}</code></p>
                    <p><strong>Return Code:</strong> ${result.returncode || 'N/A'}</p>
                `;
                
                if (result.stdout) {
                    html += `<p><strong>Output:</strong></p><div class="log-output">${result.stdout}</div>`;
                }
                
                if (result.stderr) {
                    html += `<p><strong>Error Output:</strong></p><div class="log-output">${result.stderr}</div>`;
                }
                
                if (result.error) {
                    html += `<p><strong>Error:</strong> ${result.error}</p>`;
                }
                
                resultContent.innerHTML = html;
                
            } catch (error) {
                results.style.display = 'block';
                results.className = 'result-section error';
                resultContent.innerHTML = `<h4>❌ Network Error</h4><p>${error.message}</p>`;
            }
            
            // Re-enable button
            runBtn.disabled = false;
            runBtn.textContent = '▶️ Run Bijective Map App';
        }
        
        function clearForm() {
            document.getElementById('bijectiveForm').reset();
            document.querySelectorAll('.mode-option').forEach(o => o.classList.remove('selected'));
            document.querySelector('.mode-option[data-mode="back"]').classList.add('selected');
            selectedMode = 'back';
            document.getElementById('app_mode').value = 'back';
            document.getElementById('textureOptions').style.display = 'none';
            document.getElementById('isoLinesOptions').style.display = 'none';
            document.getElementById('results').style.display = 'none';
            
            // Reset iso_lines specific fields to defaults
            document.getElementById('N').value = '5';
            document.getElementById('do_parallel').checked = true;
            
            updateCommandPreview();
        }
        
        // Search functionality
        let searchTimeout;
        document.addEventListener('DOMContentLoaded', function() {
            const searchInput = document.getElementById('searchInput');
            if (searchInput) {
                searchInput.addEventListener('input', function() {
                    clearTimeout(searchTimeout);
                    searchTimeout = setTimeout(() => filterFiles(this.value), 200);
                });
                
                // Keyboard navigation
                searchInput.addEventListener('keydown', function(e) {
                    handleSearchKeyNavigation(e);
                });
            }
        });
        
        function filterFiles(searchTerm) {
            if (!window.originalFileData) return;
            
            const fileList = document.getElementById('fileList');
            const term = searchTerm.toLowerCase().trim();
            
            if (!term) {
                // Show all files if no search term
                renderFileList(window.originalFileData.directories, window.originalFileData.files);
                return;
            }
            
            // Filter directories and files
            const filteredDirs = window.originalFileData.directories.filter(dir => 
                dir.name.toLowerCase().includes(term)
            );
            const filteredFiles = window.originalFileData.files.filter(file => 
                file.name.toLowerCase().includes(term)
            );
            
            renderFileList(filteredDirs, filteredFiles);
        }
        
        function renderFileList(directories, files) {
            const fileList = document.getElementById('fileList');
            let html = '';
            
            // Add directories
            directories.forEach(dir => {
                const icon = dir.is_parent ? '⬆️' : '📁';
                html += `
                    <div class="file-item" onclick="navigateToDirectory('${dir.path}')" data-type="dir">
                        <div class="file-icon">${icon}</div>
                        <div class="file-info">
                            <div class="file-name">${dir.name}</div>
                            <div class="file-details">Directory</div>
                        </div>
                    </div>
                `;
            });
            
            // Add files (only if not selecting directory)
            if (!isSelectingDirectory) {
                files.forEach(file => {
                    const icon = getFileIcon(file.extension);
                    const sizeStr = formatFileSize(file.size);
                    const className = file.is_supported ? 'file-item' : 'file-item unsupported';
                    
                    html += `
                        <div class="${className}" onclick="selectFile('${file.path}', '${file.name}')" data-type="file" data-path="${file.path}">
                            <div class="file-icon">${icon}</div>
                            <div class="file-info">
                                <div class="file-name">${file.name}</div>
                                <div class="file-details">${sizeStr} • ${file.extension || 'No extension'}</div>
                            </div>
                        </div>
                    `;
                });
            }
            
            fileList.innerHTML = html || '<div class="loading">No items found</div>';
        }
        
        function handleSearchKeyNavigation(e) {
            const fileItems = document.querySelectorAll('.file-item');
            const currentSelected = document.querySelector('.file-item.selected');
            let newIndex = -1;
            
            if (e.key === 'ArrowDown') {
                e.preventDefault();
                if (currentSelected) {
                    const currentIndex = Array.from(fileItems).indexOf(currentSelected);
                    newIndex = Math.min(currentIndex + 1, fileItems.length - 1);
                } else {
                    newIndex = 0;
                }
            } else if (e.key === 'ArrowUp') {
                e.preventDefault();
                if (currentSelected) {
                    const currentIndex = Array.from(fileItems).indexOf(currentSelected);
                    newIndex = Math.max(currentIndex - 1, 0);
                } else {
                    newIndex = fileItems.length - 1;
                }
            } else if (e.key === 'Enter') {
                e.preventDefault();
                if (currentSelected) {
                    if (currentSelected.dataset.type === 'dir') {
                        // Navigate to directory
                        const dirPath = currentSelected.getAttribute('onclick').match(/'([^']+)'/)[1];
                        navigateToDirectory(dirPath);
                    } else {
                        // Select file
                        const filePath = currentSelected.dataset.path;
                        const fileName = currentSelected.querySelector('.file-name').textContent;
                        selectFile(filePath, fileName);
                        // Auto-select if file is chosen
                        setTimeout(() => {
                            const selectBtn = document.getElementById('selectBtn');
                            if (!selectBtn.disabled) {
                                selectCurrentFile();
                            }
                        }, 100);
                    }
                } else if (fileItems.length > 0) {
                    // Select first item if none selected
                    fileItems[0].click();
                }
            } else if (e.key === 'Escape') {
                e.preventDefault();
                closeFileBrowser();
            }
            
            // Update selection
            if (newIndex >= 0 && fileItems[newIndex]) {
                // Remove previous selection
                document.querySelectorAll('.file-item.selected').forEach(item => {
                    item.classList.remove('selected');
                });
                
                // Add new selection
                fileItems[newIndex].classList.add('selected');
                
                // Update selectedFilePath if it's a file
                if (fileItems[newIndex].dataset.type === 'file') {
                    selectedFilePath = fileItems[newIndex].dataset.path;
                    document.getElementById('selectBtn').disabled = false;
                } else {
                    selectedFilePath = '';
                    if (!isSelectingDirectory) {
                        document.getElementById('selectBtn').disabled = true;
                    }
                }
                
                // Scroll into view
                fileItems[newIndex].scrollIntoView({ block: 'nearest' });
            }
        }
        
        // Initialize
        updateCommandPreview();
        
        // Close modal when clicking outside
        window.onclick = function(event) {
            const modal = document.getElementById('fileBrowserModal');
            if (event.target === modal) {
                closeFileBrowser();
            }
        }
    </script>
</body>
</html>
        """

def main():
    # Check if we're in the right directory
    if not Path('build/bijective_map/bijective_map_app').exists():
        print("❌ Error: bijective_map_app not found at ./build/bijective_map/bijective_map_app")
        print("Please run this script from the project root directory and ensure the application is built.")
        print("\nTo build the application:")
        print("  mkdir -p build && cd build")
        print("  cmake -DCMAKE_BUILD_TYPE=Release ..")
        print("  make bijective_map_app")
        return 1
    
    # Start server
    port = 8080
    httpd = HTTPServer(('localhost', port), BijectiveMapHandler)
    
    print(f"🌐 Starting Bijective Map Web GUI...")
    print(f"📍 Server running at: http://localhost:{port}")
    print(f"🚀 Opening browser...")
    
    # Open browser
    webbrowser.open(f'http://localhost:{port}')
    
    try:
        print(f"💡 Press Ctrl+C to stop the server")
        httpd.serve_forever()
    except KeyboardInterrupt:
        print(f"\n🛑 Server stopped")
        httpd.server_close()
        return 0

if __name__ == '__main__':
    sys.exit(main())