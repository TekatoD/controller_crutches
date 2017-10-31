#ifndef _LOCALIZATION_UTIL_H_
#define _LOCALIZATION_UTIL_H_

#include <cmath>
#include <map>
#include <ostream>

#include <minIni.h>

/*
 *  Units: meters
 */

namespace Localization {
    struct Line {
        float x1, y1, x2, y2;
        
        Line(float x1, float y1, float x2, float y2)
            : x1(x1), y1(y1), x2(x2), y2(y2)
        {
        }
        
        float Length() const {
            return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
        }
        
        bool IsPointOnLine(float px, float py, float eps=0.001)
        {
            float LengthTo = Line(x1, y1, px, py).Length();
            float LengthFrom = Line(px, py, x2, y2).Length();
            float LineLength = this->Length();
            
            return LengthTo + LengthFrom - LineLength < eps;
        }
        
        friend std::ostream& operator<<(std::ostream& out, const Line& l);
    };
    
    std::ostream& operator<<(std::ostream& out, const Line& l)
    {
        out << "(" << l.x1 << "," << l.y1 << ")->(" << l.x2 << "," << l.y2 << ")";
        return out;
    }
    
    class FieldMap {
    public:
        enum class LineType {
            CENTRAL_LINE = 0,
            FIELD_LEFT = 1,
            FIELD_RIGHT = 2,
            FIELD_TOP = 3,
            FIELD_BOTTOM = 4,
            PENALTY_LEFT_TOP = 5,
            PENALTY_LEFT_BOTTOM = 6,
            PENALTY_LEFT_HEIGHT = 7,
            PENALTY_RIGHT_TOP = 8,
            PENALTY_RIGHT_BOTTOM = 9,
            PENALTY_RIGHT_HEIGHT = 10,
        };
        
        FieldMap()
        {
        }
        
        ~FieldMap() {}
        
        void LoadIniSettings(minIni* ini)
        {
            float fw, fh, pw, ph, gh;
            
            fw = ini->getf("NewField", "field_width") / 1000.0f;
            fh = ini->getf("NewField", "field_height") / 1000.0f;
            pw = ini->getf("NewField", "penalty_width") / 1000.0f;
            ph = ini->getf("NewField", "penalty_height") / 1000.0f;
            gh = ini->getf("NewField", "gate_height") / 1000.0f;
            
            makeLines(fw, fh, pw, ph, gh);
        }
        
        void PrintFieldLines() const
        {
            for (auto& kv : m_fieldLines) {
                std::cout << (int)kv.first << " : " << kv.second << std::endl;
            }
        }
        
    private:
        std::map<LineType, Line> m_fieldLines;
        
        // hmmmm
        void makeLines(float fw, float fh, float pw, float ph, float gh)
        {
            // Middle of the field is the origin (0.0, 0.0)
            // x: left is negative, right is positive
            // y: bottom is negative, top is positive
            m_fieldLines = {
                {LineType::CENTRAL_LINE, Line(0.0f, fh/2.0f, 0.0f, -fh/2.0f)},
                {LineType::FIELD_LEFT, Line(-fw/2.0f, fh/2.0f, -fw/2.0f, -fh/2.0f)},
                {LineType::FIELD_RIGHT, Line(fw/2.0f, fh/2.0f, fw/2.0f, -fh/2.0f)},
                {LineType::FIELD_TOP, Line(-fw/2.0f, fh/2.0f, fw/2.0f, fh/2.0f)},
                {LineType::FIELD_BOTTOM, Line(-fw/2.0f, -fh/2.0f, fw/2.0f, -fh/2.0f)},
                {LineType::PENALTY_LEFT_TOP, Line(-fw/2.0f, ph/2.0f, (-fw/2.0f)+pw, ph/2.0f)},
                {LineType::PENALTY_LEFT_BOTTOM, Line(-fw/2.0f, -ph/2.0f, (-fw/2.0f)+pw, -ph/2.0f)},
                {LineType::PENALTY_LEFT_HEIGHT, Line((-fw/2.0f)+pw, ph/2.0f, (-fw/2.0f)+pw, -ph/2.0f)},
                {LineType::PENALTY_RIGHT_TOP, Line((fw/2.0f)-pw, ph/2.0f, fw/2.0f, ph/2.0f)},
                {LineType::PENALTY_RIGHT_BOTTOM, Line((fw/2.0f)-pw, -ph/2.0f, fw/2.0f, -ph/2.0f)},
                {LineType::PENALTY_RIGHT_HEIGHT, Line((fw/2.0f)-pw, ph/2.0f, (fw/2.0f)-pw, -ph/2.0f)}
            };
        }
    };

};

#endif