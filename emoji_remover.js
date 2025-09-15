const fs = require('fs');
const path = require('path');

// Regex to match emojis (covers a wide range of Unicode emoji blocks)
const emojiRegex = /[\u{1F600}-\u{1F64F}]|[\u{1F300}-\u{1F5FF}]|[\u{1F680}-\u{1F6FF}]|[\u{1F1E0}-\u{1F1FF}]|[\u{2600}-\u{26FF}]|[\u{2700}-\u{27BF}]|[\u{1f926}-\u{1f937}]|[\u{10000}-\u{10FFFF}]|[\u{1f1f2}-\u{1f1f4}]|[\u{1f1e6}-\u{1f1ff}]|[\u{1f191}-\u{1f19a}]|[\u{1f232}-\u{1f23a}]|[\u{1f250}-\u{1f251}]|[\u{1f300}-\u{1f320}]|[\u{1f330}-\u{1f335}]|[\u{1f337}-\u{1f37c}]|[\u{1f380}-\u{1f393}]|[\u{1f3a0}-\u{1f3ca}]|[\u{1f3cf}-\u{1f3d3}]|[\u{1f3e0}-\u{1f3f0}]|[\u{1f400}-\u{1f43f}]|[\u{1f440}-\u{1f4fc}]|[\u{1f4ff}-\u{1f53d}]|[\u{1f550}-\u{1f567}]|[\u{1f5fb}-\u{1f640}]|[\u{1f645}-\u{1f64f}]|[\u{1f680}-\u{1f6c5}]|[\u{1f6cc}]|[\u{1f6d0}-\u{1f6d2}]|[\u{1f6eb}-\u{1f6ec}]|[\u{1f910}-\u{1f918}]|[\u{1f980}-\u{1f984}]|[\u{1f9c0}]|[\u{1f9d0}-\u{1f9d2}]/gu;

// Allowed file extensions for text files
const allowedExtensions = ['.js', '.dart', '.json', '.md', '.yaml', '.yml', '.py', '.sh', '.txt', '.log', '.html', '.css', '.scss', '.ts', '.jsx', '.tsx', '.xml', '.svg'];

// Function to remove emojis from a string
function removeEmojis(text) {
  return text.replace(emojiRegex, '');
}

// Function to process a directory recursively
function processDirectory(dirPath) {
  const items = fs.readdirSync(dirPath, { withFileTypes: true });

  for (const item of items) {
    const fullPath = path.join(dirPath, item.name);

    if (item.isDirectory()) {
      // Recurse into subdirectories
      processDirectory(fullPath);
    } else if (item.isFile()) {
      const ext = path.extname(item.name).toLowerCase();
      if (allowedExtensions.includes(ext)) {
        try {
          // Read file content
          const content = fs.readFileSync(fullPath, 'utf8');
          // Remove emojis
          const cleanedContent = removeEmojis(content);
          // Write back if changed
          if (content !== cleanedContent) {
            fs.writeFileSync(fullPath, cleanedContent, 'utf8');
            console.log(`Emojis removed from: ${fullPath}`);
          }
        } catch (error) {
          console.error(`Error processing file ${fullPath}: ${error.message}`);
        }
      }
    }
  }
}

// Main execution
console.log('Starting emoji removal from frontend and backend files...');

try {
  processDirectory('./frontend');
  processDirectory('./backend');
  console.log('Emoji removal completed.');
} catch (error) {
  console.error(`Error during processing: ${error.message}`);
}
