package com.spin.dynamicCalibration.impl;

import com.ur.urcap.api.contribution.ProgramNodeContribution;
import com.ur.urcap.api.contribution.program.ProgramAPIProvider;
import com.ur.urcap.api.domain.URCapAPI;
import com.ur.urcap.api.domain.script.ScriptWriter;
import com.ur.urcap.api.domain.undoredo.UndoRedoManager;
import com.ur.urcap.api.domain.undoredo.UndoableChanges;
import com.ur.urcap.api.domain.userinteraction.keyboard.KeyboardInputCallback;
import com.ur.urcap.api.domain.userinteraction.keyboard.KeyboardInputFactory;
import com.ur.urcap.api.domain.userinteraction.keyboard.KeyboardTextInput;
import com.ur.urcap.api.domain.data.DataModel;

import com.spin.dynamicCalibration.impl.Variables;

public class SendCMDProgramNodeContribution implements ProgramNodeContribution{
	Variables variables = new Variables();
	private final ProgramAPIProvider apiProvider;
	private final SendCMDProgramNodeView view;
	private final DataModel model;
	private final UndoRedoManager undoRedoManager;
	private final KeyboardInputFactory keyboardInputFactory;

	public SendCMDProgramNodeContribution(ProgramAPIProvider apiProvider, SendCMDProgramNodeView view, DataModel model) {
		this.apiProvider = apiProvider;
		this.view = view;
		this.model = model;
		this.undoRedoManager = this.apiProvider.getProgramAPI().getUndoRedoManager();
		this.keyboardInputFactory = apiProvider.getUserInterfaceAPI().getUserInteraction().getKeyboardInputFactory();
	}


	@Override
	public void openView() {view.setJTextField(getName("Message"), "Message");}


	@Override
	public void closeView() {}


	@Override
	public String getTitle() {return "Message: " + getName("Message");}


	@Override
	public boolean isDefined() {return true;}



	@Override
	public void generateScript(ScriptWriter writer) {
		// Connect
		writer.appendLine("socketStatus = socket_open(\"" + getName("IP") + "\", " + getName("PORT") + ")");
		writer.whileCondition("socketStatus == False");
		writer.appendLine("socketStatus = socket_open(\"" + getName("IP") + "\", " + getName("PORT") + ")");
		writer.sleep(1);
		writer.sync();	
		writer.end();

		// Message	
		writer.appendLine("socket_send_string(\"" + getName("Message") + "\")");
		writer.appendLine("socket_send_byte(13)");
		writer.appendLine("socket_send_byte(10)");
		writer.sleep(1);
	}


	public KeyboardTextInput getKeyboardForTextField(String textID) {
		KeyboardTextInput keyboardTextInput = keyboardInputFactory.createStringKeyboardInput();
		keyboardTextInput.setInitialValue(getName(textID));
		return keyboardTextInput;
	}


	public KeyboardInputCallback<String> getCallbackForTextField(final String jTextFieldID) {
		return new KeyboardInputCallback<String>() {
			@Override
			public void onOk(String value) {
				if ("".equals(value)) {
					setJTextFieldValue(variables.Message, jTextFieldID);
					view.setJTextField(variables.Message, jTextFieldID);
				} else {
					setJTextFieldValue(value, jTextFieldID);
					view.setJTextField(value, jTextFieldID);
				}
			}
		};
	}


	private String getName(String ID) {
		if ("IP".equals(ID)) {
			return model.get(variables.IPv4, variables.IPv4);
		} else if ("PORT".equals(ID)) {
			return model.get(variables.Port, variables.Port);
		} else {		
			return model.get(variables.Message,  variables.Message);
		}
	}


	private void setJTextFieldValue(final String value, final String jTextFieldID) {
		undoRedoManager.recordChanges(new UndoableChanges() {
			@Override
			public void executeChanges() {
				if ("".equals(value)) {
					model.remove(variables.Message);
				} else {
					model.set(variables.Message, value);
				}
			}
		});
	}
}
