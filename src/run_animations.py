import ML
import sys

def main():
    if len(sys.argv) < 3:
        exit("Usage: python run_animations.py [sim|anim] [ca|idm]")
    sim_or_anim = sys.argv[1]
    ca_or_idm = sys.argv[2]

    if sim_or_anim == "sim":
        if ca_or_idm == "ca":
            ML.run_simulation_ca()
        else:
            ML.run_simulation_idm()
    else:
        if ca_or_idm == "ca":
            ML.run_animation_ca()
        else:
            ML.run_animation_idm()

if __name__ == '__main__':
    main()
