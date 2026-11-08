"""Apply the Phase 1 opacity fix to the pinned gz-rendering 8.2.3 source.

No class layout or public ABI changes. Unknown/undecodable textures retain the
original alpha path. Inspect decoded texels, not filename or alpha-channel
presence: RGBA textures may still be completely opaque.
"""
import hashlib
from pathlib import Path
import sys

EXPECTED = '3620eb810c46906ebb2af2ca0e4f83554d952fcf298d45634be8e249c5f2c5a8'
anchor = '  if (alpha < 1 || this->textureAlphaEnabled)'
replacement = '''  // Phase 1: Gazebo enables texture alpha even for opaque albedo maps.
  // Keep the requested alpha flag, but only blend when texels need it. Do
  // this at material updates, never during rendering or by GPU readback.
  bool textureHasAlpha = this->textureAlphaEnabled && this->HasTexture();
  if (textureHasAlpha)
  {
    if (this->textureData)
    {
      const auto rgba = this->textureData->RGBAData();
      if (!rgba.empty())
      {
        textureHasAlpha = false;
        for (size_t i = 3; i < rgba.size(); i += 4)
          if (rgba[i] != 255) { textureHasAlpha = true; break; }
      }
    }
    else
    {
      try
      {
      Ogre::Image image;
      this->LoadOneImage(this->textureName, image);
      if (image.getWidth() && image.getHeight())
      {
        textureHasAlpha = false;
        if (Ogre::PixelUtil::hasAlpha(image.getFormat()))
          for (size_t z = 0; z < image.getDepth() && !textureHasAlpha; ++z)
            for (size_t y = 0; y < image.getHeight() && !textureHasAlpha; ++y)
              for (size_t x = 0; x < image.getWidth(); ++x)
                if (image.getColourAt(x, y, z).a < 1.0f)
                { textureHasAlpha = true; break; }
      }
      }
      catch (const Ogre::Exception &)
      {
        // Some compressed formats cannot expose individual decoded colors.
        // Preserve the original alpha path instead of failing material setup.
        textureHasAlpha = true;
      }
    }
  }

  if (alpha < 1 || textureHasAlpha)'''


def patch(original):
    if hashlib.sha256(original).hexdigest() != EXPECTED:
        raise ValueError('Unexpected OgreMaterial.cc; refusing an unreviewed patch')
    text = original.decode()
    changes = [
        (anchor, replacement),
        # Always reset alpha operations. Otherwise a previous uniform material
        # transparency can survive a texture replacement / alpha flag change.
        ('''    if (!this->textureAlphaEnabled)
    {
      this->ogreTexState->setAlphaOperation(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL,
          Ogre::LBS_CURRENT, alpha);
    }''', '''    if (textureHasAlpha)
      this->ogreTexState->setAlphaOperation(Ogre::LBX_MODULATE,
          Ogre::LBS_TEXTURE, Ogre::LBS_MANUAL, 1.0, alpha);
    else
      this->ogreTexState->setAlphaOperation(Ogre::LBX_SOURCE1,
          Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, alpha);'''),
        ('''    this->ogrePass->setSceneBlending(Ogre::SBT_REPLACE);''',
         '''    this->ogrePass->setSceneBlending(Ogre::SBT_REPLACE);
    this->ogreTexState->setAlphaOperation(Ogre::LBX_SOURCE1,
        Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, 1.0);'''),
        ('    this->SetTextureDataImpl(this->textureName, _img);',
         '    this->SetTextureDataImpl(this->textureName, _img);\n  this->UpdateTransparency();'),
        ('  this->ogreTexState->setBlank();\n  this->UpdateColorOperation();',
         '  this->ogreTexState->setBlank();\n  this->UpdateColorOperation();\n  this->UpdateTransparency();'),
    ]
    for before, after in changes:
        if text.count(before) != 1:
            raise ValueError('Unexpected pinned source shape: '+before)
        text = text.replace(before, after, 1)
    return text


if __name__ == '__main__':
    source = Path(sys.argv[1]) / 'ogre/src/OgreMaterial.cc'
    source.write_text(patch(source.read_bytes()))
