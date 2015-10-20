import sys,os

# Add the conversion script to our path
p = os.path.abspath(__file__)
print os.path.join(os.path.dirname(p), "../")
sys.path.append(os.path.join(os.path.dirname(p), "../../"))

slugs_converter_path = os.path.join(os.path.join(os.path.dirname(p), "../../"), "etc", "slugs", "tools")
sys.path.insert(0, slugs_converter_path)
from translateFromLTLMopLTLFormatToSlugsFormat import performConversion

def prepareSlugsInput(proj_dir):
    """ Convert from JTLV input format (.smv+.ltl) to Slugs input format (.slugsin)
        using the script provided by Slugs.

        This is a stop-gap fix; eventually we should just produce the input
        directly instead of using the conversion script. """

    # Call the conversion script
    with open(proj_dir + ".slugsin", "w+") as f:
        # TODO: update performConversion so we don't have to do stdout redirection
        sys.stdout = f
        performConversion(proj_dir + ".smv", proj_dir + ".ltl")
        sys.stdout = sys.__stdout__



if __name__ == '__main__':
    if len(sys.argv) > 1:
        file_dir = sys.argv[1]
    else:
        file_dir = os.path.join(os.path.join(os.path.dirname(p), "../../"), "examples/firefighting/firefighting")
    prepareSlugsInput(file_dir)
    print "Converted " + file_dir + '.ltl to ' + file_dir + '.slugsin!\n'