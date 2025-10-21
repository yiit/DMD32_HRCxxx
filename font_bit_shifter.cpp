#include <iostream>
#include <vector>
#include <iomanip>

// Font bit shifter algorithm
class FontBitShifter {
private:
    std::vector<uint8_t> fontData;
    int rows;
    int cols;
    
public:
    FontBitShifter(int numRows, int numCols) : rows(numRows), cols(numCols) {
        fontData.resize(rows * cols);
    }
    
    // Set font data from array
    void setFontData(const std::vector<uint8_t>& data) {
        fontData = data;
    }
    
    // Shift all columns by specified bits to the right (downward)
    void shiftColumnsRight(int shiftBits) {
        std::vector<uint8_t> newData(rows * cols, 0);
        
        for (int col = 0; col < cols; col++) {
            // Extract 3 bytes from current column (rows 0, 1, 2)
            uint32_t columnData = 0;
            
            // Combine bytes: row2 | row1 | row0 (24-bit value)
            for (int row = 0; row < rows; row++) {
                columnData |= (uint32_t)fontData[row * cols + col] << (8 * (rows - 1 - row));
            }
            
            // Shift right by specified bits
            columnData >>= shiftBits;
            
            // Split back into bytes
            for (int row = 0; row < rows; row++) {
                newData[row * cols + col] = (columnData >> (8 * (rows - 1 - row))) & 0xFF;
            }
        }
        
        fontData = newData;
    }
    
    // Print font data in binary format
    void printBinary() {
        for (int row = 0; row < rows; row++) {
            std::cout << "    ";
            for (int col = 0; col < cols; col++) {
                std::cout << "B";
                for (int bit = 7; bit >= 0; bit--) {
                    std::cout << ((fontData[row * cols + col] >> bit) & 1);
                }
                if (col < cols - 1) std::cout << ", ";
            }
            std::cout << ", " << std::endl;
        }
        std::cout << std::endl;
    }
    
    // Print font data in hex format
    void printHex() {
        for (int row = 0; row < rows; row++) {
            std::cout << "    ";
            for (int col = 0; col < cols; col++) {
                std::cout << "0x" << std::hex << std::uppercase 
                         << std::setfill('0') << std::setw(2) 
                         << (int)fontData[row * cols + col];
                if (col < cols - 1) std::cout << ", ";
            }
            std::cout << ", " << std::endl;
        }
        std::cout << std::endl;
    }
    
    // Get current font data
    std::vector<uint8_t> getFontData() const {
        return fontData;
    }
};

// Example usage for '0' character
int main() {
    // Original '0' character data (3 rows x 13 columns)
    std::vector<uint8_t> originalData = {
        // Row 0 (top)
        0x00, 0xC0, 0xE0, 0xF0, 0x78, 0x38, 0x38, 0x38, 0x78, 0xF0, 0xF0, 0xC0, 0x00,
        // Row 1 (middle) 
        0xFE, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF,
        // Row 2 (bottom)
        0x01, 0x07, 0x0F, 0x1F, 0x3C, 0x38, 0x38, 0x38, 0x3C, 0x1F, 0x0F, 0x07, 0x00
    };
    
    FontBitShifter shifter(3, 13);  // 3 rows, 13 columns
    shifter.setFontData(originalData);
    
    std::cout << "Original '0' character data:" << std::endl;
    shifter.printBinary();
    
    // Shift 4 bits to the right (downward)
    shifter.shiftColumnsRight(4);
    
    std::cout << "After 4-bit right shift (moved down 4 pixels):" << std::endl;
    shifter.printBinary();
    
    std::cout << "Hex format:" << std::endl;
    shifter.printHex();
    
    return 0;
}

/*
ALGORITHM EXPLANATION:
1. Font data is stored as 3 rows x 13 columns matrix
2. Each column represents vertical pixels for one character column
3. For each column:
   - Combine 3 bytes into 24-bit value (row2|row1|row0)
   - Shift right by N bits (moves character down N pixels)
   - Split back into 3 separate bytes
4. Result: Character moved down by N pixels

USAGE FOR OTHER CHARACTERS:
- Change originalData array with target character data
- Adjust rows/cols parameters if character size differs
- Call shiftColumnsRight(N) with desired pixel shift amount
*/