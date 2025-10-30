#!/usr/bin/env python3
"""
Serial.print/printf mesajlarını DEBUG makrolarına çeviren script
"""

import re

def convert_debug_messages():
    with open('src/main.cpp', 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Serial.println -> DEBUG_PRINTLN
    content = re.sub(r'Serial\.println\(([^)]+)\);', r'DEBUG_PRINTLN(\1);', content)
    
    # Serial.print -> DEBUG_PRINT  
    content = re.sub(r'Serial\.print\(([^)]+)\);', r'DEBUG_PRINT(\1);', content)
    
    # Serial.printf -> DEBUG_PRINTF
    content = re.sub(r'Serial\.printf\(([^)]+)\);', r'DEBUG_PRINTF(\1);', content)
    
    # Kritik error mesajları geri çevir (korunmalı)
    critical_patterns = [
        'AP-STA MOD BASARISIZ',
        'AP-STA SSID BASARISIZ', 
        'AP-STA KOFIGURASYON BASARISIZ'
    ]
    
    for pattern in critical_patterns:
        content = content.replace(f'DEBUG_PRINTLN("{pattern}!")', f'Serial.println("{pattern}!")')
    
    # Makro tanımlarındaki Serial kullanımlarını koru
    content = content.replace('DEBUG_PRINT(x) if(debugEnabled) { DEBUG_PRINT(x); }', 
                             'DEBUG_PRINT(x) if(debugEnabled) { Serial.print(x); }')
    content = content.replace('DEBUG_PRINTLN(x) if(debugEnabled) { DEBUG_PRINTLN(x); }', 
                             'DEBUG_PRINTLN(x) if(debugEnabled) { Serial.println(x); }')
    content = content.replace('DEBUG_PRINTF(...) if(debugEnabled) { DEBUG_PRINTF(__VA_ARGS__); }', 
                             'DEBUG_PRINTF(...) if(debugEnabled) { Serial.printf(__VA_ARGS__); }')
    
    with open('src/main.cpp', 'w', encoding='utf-8') as f:
        f.write(content)
    
    print("✅ Debug mesajları başarıyla çevrildi!")

if __name__ == "__main__":
    convert_debug_messages()