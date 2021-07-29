/* -*-c++-*- OpenSceneGraph - Copyright (C) 1998-2013 Robert Osfield
 *
 * This library is open source and may be redistributed and/or modified under
 * the terms of the OpenSceneGraph Public License (OSGPL) version 0.0 or
 * (at your option) any later version.  The full license is in LICENSE file
 * included with this distribution, and on the openscenegraph.org website.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * OpenSceneGraph Public License for more details.
*/

#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>

#include "V8ScriptEngine.h"

class ReaderWriterV8 : public osgDB::ReaderWriter
{
    public:

        ReaderWriterV8()
        {
            supportsExtension("v8","JavaScript");
            supportsExtension("js","JavaScript");
        }

        virtual const char* className() const { return "V8 JavaScript ScriptEngine plugin"; }

        virtual ReadResult readObject(std::istream& fin, const osgDB::ReaderWriter::Options* options =NULL) const
        {
            return readScript(fin, options);
        }

        virtual ReadResult readObject(const std::string& file, const osgDB::ReaderWriter::Options* options =NULL) const
        {
            if (file=="ScriptEngine.V8") return new v8::V8ScriptEngine();
            if (file=="ScriptEngine.js") return new v8::V8ScriptEngine();

            return readScript(file);
        }

        virtual ReadResult readScript(std::istream& fin,const osgDB::ReaderWriter::Options* options =NULL) const
        {
            osg::ref_ptr<osg::Script> script = new osg::Script;
            script->setLanguage("js");

            std::string str;
            while(fin)
            {
                int c = fin.get();
                if (c>=0 && c<=255)
                {
                    str.push_back(c);
                }
            }
            script->setScript(str);

            return script.release();
        }

        virtual ReadResult readScript(const std::string& file, const osgDB::ReaderWriter::Options* options =NULL) const
        {
            std::string ext = osgDB::getLowerCaseFileExtension(file);
            if (!acceptsExtension(ext)) return ReadResult::FILE_NOT_HANDLED;

            std::string fileName = osgDB::findDataFile( file, options );
            if (fileName.empty()) return ReadResult::FILE_NOT_FOUND;

            osgDB::ifstream istream(fileName.c_str(), std::ios::in);
            if(!istream) return ReadResult::FILE_NOT_HANDLED;

            return readScript(istream, options);
        }
};

// now register with Registry to instantiate the above
// reader/writer.
REGISTER_OSGPLUGIN(v8, ReaderWriterV8)
