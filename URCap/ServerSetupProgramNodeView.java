package com.spin.dynamicCalibration.impl;

import java.awt.Component;
import java.awt.Dimension;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;

import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;

import com.ur.urcap.api.contribution.ContributionProvider;
import com.ur.urcap.api.contribution.ViewAPIProvider;
import com.ur.urcap.api.contribution.program.swing.SwingProgramNodeView;
import com.ur.urcap.api.domain.userinteraction.keyboard.KeyboardTextInput;

public class ServerSetupProgramNodeView implements SwingProgramNodeView<ServerSetupProgramNodeContribution>{
	private JTextField JTextFieldIP;
	private JTextField JTextFieldPORT;

	public ServerSetupProgramNodeView(ViewAPIProvider apiProvider) {
		this.JTextFieldIP = new JTextField();
		this.JTextFieldPORT = new JTextField();
	}

	@Override
	public void buildUI(JPanel panel, ContributionProvider<ServerSetupProgramNodeContribution> provider) {
		panel.setLayout(new BoxLayout(panel, BoxLayout.Y_AXIS));

		panel.add(createVerticalSpacer(10));
		panel.add(createDescription("Setup specifications for the Client."));
		panel.add(createVerticalSpacer(10));
		panel.add(createInputLine("Set IP:			", provider, JTextFieldIP));
		panel.add(createVerticalSpacer(10));
		panel.add(createInputLine("Set Port:		", provider, JTextFieldPORT));
		panel.add(createVerticalSpacer(10));
		panel.add(createDescription("If nothing is spcified, the default setteings are used."));
	}

	

	private Box createDescription(final String desc) {
		Box box = Box.createHorizontalBox();
		box.setAlignmentX(Component.LEFT_ALIGNMENT);

		JLabel label = new JLabel(desc);

		box.add(label);
		return box;
	}

	private Box createInputLine(final String desc, final ContributionProvider<ServerSetupProgramNodeContribution> provider,
			final JTextField jTextField) {
		Box inputBox = Box.createHorizontalBox();
		inputBox.setAlignmentX(Component.LEFT_ALIGNMENT);
		inputBox.add(new JLabel(desc));
		inputBox.add(createHorizontalSpacing(10));

		jTextField.setFocusable(false);
		jTextField.setPreferredSize(new Dimension(200, 24));
		jTextField.setMaximumSize(jTextField.getPreferredSize());
		jTextField.addMouseListener(new MouseAdapter() {

			@Override
			public void mousePressed(MouseEvent e) {
				if (jTextField == JTextFieldIP) {
					KeyboardTextInput keyboardTextInput = provider.get().getKeyboardForTextField("IP");
					keyboardTextInput.show(jTextField, provider.get().getCallbackForTextField("IP"));
				} else {
					KeyboardTextInput keyboardTextInput = provider.get().getKeyboardForTextField("PORT");
					keyboardTextInput.show(jTextField, provider.get().getCallbackForTextField("PORT"));
				}
			}
		});

		inputBox.add(jTextField);
		return inputBox;
	}

	private Component createVerticalSpacer(int height) {
		return Box.createRigidArea(new Dimension(0, height));
	}

	public void setJTextField(final String value, final String jTextFieldID) {
		if ("IP".equals(jTextFieldID)) {
			JTextFieldIP.setText(value);
		} else {
			JTextFieldPORT.setText(value);
		}
	}

	private Component createHorizontalSpacing(int horizontalSpacing) {
		return Box.createRigidArea(new Dimension(horizontalSpacing, 0));
	}
}