#ifndef FONT_FUN_H_
#define FONT_FUN_H_

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <string>
#include <cstdlib>

#include <ft2build.h>
#include FT_FREETYPE_H
#include <ftoutln.h>

#include <Eigen/Core>
#include "interpolators.h"

struct DecompositionData {
  double scale;
  FT_Vector pen;
  FT_Vector last_point;
  std::vector<PathGen*> splines;
};

int MoveTo(const FT_Vector* to, void* fp);
int LineTo(const FT_Vector* to, void* fp);
int ConicTo(const FT_Vector* p1, const FT_Vector* to, void* fp);
int CubicTo(const FT_Vector* p1, const FT_Vector* p2, const FT_Vector* to, void* fp);

class FreetypeSplineInterface {
public:
  FT_Library library;
  FT_Face face;
  FT_GlyphSlot slot;
  FT_Error error;
  FT_Outline_Funcs funcs;

  // Font File
  std::string filename;

  FreetypeSplineInterface(std::string font_path) : filename(font_path) {
    funcs.move_to = (FT_Outline_MoveTo_Func)&MoveTo;
    funcs.line_to = (FT_Outline_LineTo_Func)&LineTo;
    funcs.conic_to = (FT_Outline_ConicTo_Func)&ConicTo;
    funcs.cubic_to = (FT_Outline_CubicTo_Func)&CubicTo;
    funcs.shift = 0;
    funcs.delta = 0;

    error = FT_Init_FreeType( &library );
    error = FT_New_Face( library, filename.c_str(), 0, &face );
    slot = face->glyph;
  }

  ~FreetypeSplineInterface() {
    FT_Done_Face(face);
    FT_Done_FreeType(library);
  }

  std::vector<PathGen*> GetSplines(std::string text, double scale) {
    printf("Making splines for: %s\n", text.c_str());
    FT_UInt previous_glyph_index = 0;
    FT_UInt current_glyph_index = 0;

    DecompositionData data;
    data.scale = scale;
    data.pen.x = 0;
    data.pen.y = 0;

    for (uint n = 0; n < text.size(); n++ ) {
      current_glyph_index = FT_Get_Char_Index( face, text[n] );
      error = FT_Load_Glyph( face, current_glyph_index, FT_LOAD_NO_SCALE );
      if (face->glyph->format != FT_GLYPH_FORMAT_OUTLINE) {
        printf("Need a font with outlines.\n");
        return data.splines;
      }

      if ( previous_glyph_index && current_glyph_index ) {
        FT_Vector  delta;
        error = FT_Get_Kerning( face, previous_glyph_index, current_glyph_index, FT_KERNING_UNSCALED, &delta );
        if ( error ) {
          printf("Kerning errors\n");
        }
        data.pen.x += delta.x;
        data.pen.y += delta.y;
      }

      // trace outline of the glyph
      error = FT_Outline_Decompose(&slot->outline, &funcs, (void*)&data);
      if ( error ) {
        printf("Decomposition errors\n");
      }

      data.pen.x += slot->advance.x;
      data.pen.y += slot->advance.y;
      previous_glyph_index = current_glyph_index;
    }
    return data.splines;
  }
};

void FontFun(int argc, char**  argv);

#endif // FONT_FUN_H_
