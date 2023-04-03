import jinja2
import pdfkit
from datetime import datetime
import pickle


def create_pdf_report(subj_id, num_trial, date, pth):
#def create_pdf_report(subj_id, num_trial, date, pth, image):

    with open(f"{pth}/metrics.pkl", 'rb') as f:
        results = pickle.load(f)

    context = {'subject_id': subj_id, 'num_trial': num_trial, 'date': date, 
               'full_steps': results['Full TUG']['nsteps'], 'full_walking_time': str(format(results['Full TUG']['walk_time'],'.2f')), 'full_TUG_time': str(format(results['Full TUG']['ex_time'],'.2f')), 
               'mean_step_length': str(format(results['Full TUG']['step_length'],'.2f')), 'mean_step_width': str(format(results['Full TUG']['step_width'],'.2f')), 'full_frequency': str(format(results['Full TUG']['frequency'],'.2f')), 
               'full_speed': str(format(results['Full TUG']['speed'],'.2f')), 'full_acceleration': str(format(results['Full TUG']['acceleration'],'.2f')), 'full_speed_z': str(format(results['Full TUG']['speed_z'],'.2f')), 
               'full_acceleration_z': str(format(results['Full TUG']['acceleration_z'],'.2f')), 
               'standing_time': str(format(results['Standing']['ex_time'],'.2f')), 'standing_speed_z': str(format(results['Standing']['speed_z'],'.2f')), 
               'standing_acceleration_z': str(format(results['Standing']['acceleration_z'],'.2f')), 
               'sitting_time': str(format(results['Sitting']['ex_time'],'.2f')), 'sitting_speed_z': str(format(results['Sitting']['speed_z'],'.2f')), 
               'sitting_acceleration_z': str(format(results['Sitting']['acceleration_z'],'.2f')),
                'walk_f_steps': results['Walking forward']['nsteps'], 'walk_f_time': str(format(results['Walking forward']['ex_time'],'.2f')), 'walk_f_frequency': str(format(results['Walking forward']['frequency'],'.2f')), 
               'walk_f_speed': str(format(results['Walking forward']['speed'],'.2f')), 'walk_f_acceleration': str(format(results['Walking forward']['acceleration'],'.2f')), 'walk_f_speed_z': str(format(results['Walking forward']['speed_z'],'.2f')), 
               'walk_f_acceleration_z': str(format(results['Walking forward']['acceleration_z'],'.2f')),
               'walk_b_steps': results['Walking backward']['nsteps'], 'walk_b_time': str(format(results['Walking backward']['ex_time'],'.2f')), 'walk_b_frequency': str(format(results['Walking backward']['frequency'],'.2f')), 
               'walk_b_speed': str(format(results['Walking backward']['speed'],'.2f')), 'walk_b_acceleration': str(format(results['Walking backward']['acceleration'],'.2f')), 'walk_b_speed_z': str(format(results['Walking backward']['speed_z'],'.2f')), 
               'walk_b_acceleration_z': str(format(results['Walking backward']['acceleration_z'],'.2f')),
               'turn1_steps': results['Turning1']['nsteps'], 'turn1_time': str(format(results['Turning1']['ex_time'],'.2f')), 'turn1_frequency': str(format(results['Turning1']['frequency'],'.2f')), 
               'turn1_speed': str(format(results['Turning1']['speed'],'.2f')), 'turn1_acceleration': str(format(results['Turning1']['acceleration'],'.2f')), 'turn1_speed_z': str(format(results['Turning1']['speed_z'],'.2f')), 
               'turn1_acceleration_z': str(format(results['Turning1']['acceleration_z'],'.2f')),
               'turn2_steps': results['Turning2']['nsteps'], 'turn2_time': str(format(results['Turning2']['ex_time'],'.2f')), 'turn2_frequency': str(format(results['Turning2']['frequency'],'.2f')), 
               'turn2_speed': str(format(results['Turning2']['speed'],'.2f')), 'turn2_acceleration': str(format(results['Turning2']['acceleration'],'.2f')), 'turn2_speed_z': str(format(results['Turning2']['speed_z'],'.2f')), 
               'turn2_acceleration_z': str(format(results['Turning2']['acceleration_z'],'.2f'))}#, 'image': image}

    template_loader = jinja2.FileSystemLoader('./')
    template_env = jinja2.Environment(loader=template_loader)

    html_template = 'report_template.html'
    template = template_env.get_template(html_template)
    output_text = template.render(context)

    config = pdfkit.configuration(wkhtmltopdf='/usr/bin/wkhtmltopdf')
    output_pdf = f'{pth}/R1_TUG_report.pdf'
    #pdfkit.from_string(output_text, output_pdf, configuration=config, css='style.css')
    pdfkit.from_string(output_text, output_pdf, configuration=config)





def create_txt_report(subj_id, num_trial, pth):

    phases_names= ['Standing', 'Walking forward', 'Turning1', 'Walking backward', 'Turning2', 'Sitting']

    with open(f"{pth}/metrics.pkl", 'rb') as f:
        results = pickle.load(f)

    output_txt = f'{pth}/R1_TUG_report.txt'

    with open (output_txt, 'w') as file:   #Durata delle sottofasi: alzata, cammino andata, curva, cammino ritorno, seduta, numero di passi totali e nelle varie fasi, cadenza (passi al minuto). 
        for i in range(6):

            file.writelines('Phase: '+ phases_names[i] + '\n')    
            file.writelines('Steps: '+ str(results[phases_names[i]]['nsteps']) + '\n') 
            file.writelines('Execution time: '+ str(format(results[phases_names[i]]['ex_time'],'.2f')) +' s\n') 
            file.writelines('Frequency: '+ str(format(results[phases_names[i]]['frequency'],'.2f')) +' step/s\n') 
            file.writelines('Velocity: '+ str(format(results[phases_names[i]]['speed'],'.2f')) +' m/s\n') 
            file.writelines('Acceleration: '+ str(format(results[phases_names[i]]['acceleration'],'.2f')) +' m/s^2\n') 
            file.writelines('Velocity in z: '+ str(format(results[phases_names[i]]['speed_z'],'.2f')) +' m/s\n') 
            file.writelines('Acceleration in z: '+ str(format(results[phases_names[i]]['acceleration_z'],'.2f')) +' m/s^2\n') 
            file.writelines('\n')

        file.writelines('Full TUG\n')    
        file.writelines('Steps: '+ str(results['Full TUG']['nsteps'])  +'\n') 
        file.writelines('Walking time: '+ str(format(results['Full TUG']['walk_time'],'.2f')) +' s\n')
        file.writelines('TUG time: '+ str(format(results['Full TUG']['ex_time'],'.2f')) +' s\n') 
        file.writelines('Mean step length: '+ str(format(results['Full TUG']['step_length'],'.2f')) +' m\n') 
        file.writelines('Mean step width: '+ str(format(results['Full TUG']['step_width'],'.2f')) +' m\n') 
        file.writelines('Frequency: '+ str(format(results['Full TUG']['frequency'],'.2f')) +' step/s\n')  
        file.writelines('Velocity: '+ str(format(results['Full TUG']['speed'],'.2f')) +' m/s\n') 
        file.writelines('Acceleration: '+ str(format(results['Full TUG']['acceleration'],'.2f')) +' m/s^2\n') 
        file.writelines('Velocity in z: '+ str(format(results['Full TUG']['speed_z'],'.2f')) +' m/s\n') 
        file.writelines('Acceleration in z: '+ str(format(results['Full TUG']['acceleration_z'],'.2f')) +' m/s^2\n') 

    file.close